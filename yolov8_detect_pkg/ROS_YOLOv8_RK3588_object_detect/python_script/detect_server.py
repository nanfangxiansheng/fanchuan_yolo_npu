#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO检测图片服务端
监控检测结果目录的origin子目录，当有新原图时通过TCP发送给客户端
同时监听PX4飞控连接，获取经纬度坐标随图片一起发送

使用方法：
python3 detect_server.py [--port 8888] [--dir /home/orangepi/detected-results] [--px4-port 6666]
"""

import os
import socket
import threading
import time
import json
import argparse
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ImageHandler(FileSystemEventHandler):
    def __init__(self, server):
        self.server = server

    def on_created(self, event):
        if not event.is_directory and event.src_path.endswith('.jpg'):
            time.sleep(0.1)  # 等待文件写入完成
            self.server.send_image(event.src_path)

class DetectServer:
    def __init__(self, port, base_dir, px4_port):
        self.port = port
        self.base_dir = base_dir
        self.px4_port = px4_port
        # 只监控origin子目录（原图）
        self.origin_dir = os.path.join(base_dir, 'origin')
        self.clients = []
        self.lock = threading.Lock()
        self.running = True
        # PX4坐标相关
        self.latest_coord = "N/A"
        self.prev_coord = None
        self.coord_lock = threading.Lock()

        # 确保目录存在
        os.makedirs(self.origin_dir, exist_ok=True)

    def px4_listener(self):
        """监听PX4飞控连接，持续获取经纬度坐标"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.px4_port))
        server.listen(1)
        server.settimeout(1.0)
        print(f"[PX4] listener started on port {self.px4_port}")

        while self.running:
            try:
                client_sock, addr = server.accept()
                print(f"[PX4] connected: {addr}")
                buffer = b""
                try:
                    while self.running:
                        data = client_sock.recv(1024)
                        if not data:
                            break
                        buffer += data
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            try:
                                msg = json.loads(line.decode('utf-8'))
                                coord = msg.get('flight_coord', 'N/A')
                                with self.coord_lock:
                                    self.latest_coord = coord
                                if coord != self.prev_coord:
                                    print(f"[PX4] coord changed: {coord}")
                                    self.prev_coord = coord
                            except json.JSONDecodeError:
                                continue
                except Exception as e:
                    print(f"[PX4] connection lost: {addr}")
                finally:
                    client_sock.close()
            except socket.timeout:
                continue
        server.close()

    def send_image(self, file_path):
        """发送图片和坐标给所有连接的客户端"""
        try:
            with open(file_path, 'rb') as f:
                image_data = f.read()

            filename = os.path.basename(file_path)
            filesize = len(image_data)

            with self.coord_lock:
                coord = self.latest_coord

            with self.lock:
                clients_to_remove = []
                for client, addr in self.clients:
                    try:
                        # 发送文件名长度和文件名
                        name_bytes = filename.encode('utf-8')
                        client.sendall(len(name_bytes).to_bytes(4, 'big'))
                        client.sendall(name_bytes)

                        # 发送坐标长度和坐标
                        coord_bytes = coord.encode('utf-8')
                        client.sendall(len(coord_bytes).to_bytes(4, 'big'))
                        client.sendall(coord_bytes)

                        # 发送文件大小和文件内容
                        client.sendall(filesize.to_bytes(8, 'big'))
                        client.sendall(image_data)
                        print(f"[SEND] {filename} | coord: {coord} | size: {filesize} bytes -> {addr}")
                    except Exception as e:
                        print(f"[SEND] failed: {e}")
                        clients_to_remove.append((client, addr))

                # 移除断开的客户端
                for client, addr in clients_to_remove:
                    self.clients.remove((client, addr))
                    client.close()

        except Exception as e:
            print(f"[ERROR] reading image: {e}")

    def handle_client(self, client_socket, addr):
        """处理单个客户端连接"""
        print(f"[CLIENT] connected: {addr}")
        with self.lock:
            self.clients.append((client_socket, addr))

        try:
            while self.running:
                # 保持连接活跃
                time.sleep(1)
        except Exception as e:
            print(f"[CLIENT] disconnected: {addr}")
        finally:
            with self.lock:
                self.clients = [(s, a) for s, a in self.clients if s != client_socket]
                client_socket.close()

    def start(self):
        """启动服务端"""
        # 启动PX4监听线程
        px4_thread = threading.Thread(target=self.px4_listener)
        px4_thread.daemon = True
        px4_thread.start()

        # 启动文件监控（只监控origin目录）
        event_handler = ImageHandler(self)
        observer = Observer()
        observer.schedule(event_handler, self.origin_dir, recursive=False)
        observer.start()
        print(f"[MONITOR] watching: {self.origin_dir}")

        # 启动TCP服务器
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', self.port))
        server_socket.listen(5)
        server_socket.settimeout(1.0)
        print(f"[SERVER] listening on port {self.port}")

        try:
            while self.running:
                try:
                    client_socket, addr = server_socket.accept()
                    client_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    continue
        except KeyboardInterrupt:
            print("\n[SERVER] stopping...")
        finally:
            self.running = False
            observer.stop()
            observer.join()
            server_socket.close()

            # 关闭所有客户端连接
            with self.lock:
                for client, addr in self.clients:
                    client.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='YOLO检测图片服务端（含PX4坐标）')
    parser.add_argument('--port', type=int, default=8888, help='监听端口')
    parser.add_argument('--dir', type=str, default='/home/orangepi/detected-results', help='检测图片基础目录')
    parser.add_argument('--px4-port', type=int, default=6667, help='PX4飞控监听端口')
    args = parser.parse_args()

    server = DetectServer(args.port, args.dir, args.px4_port)
    server.start()
