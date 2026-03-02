#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取 FZ Token 工具
使用 RSA 公钥加密密码后调用登录 API 获取 Authorization Token
支持定时任务自动刷新 token 并保存到缓存
"""

import base64
import json
import os
import sys
import threading
import time
from typing import Optional

import requests
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend
from dotenv import load_dotenv

# 加载环境变量
load_dotenv("config.env")

# Token 缓存文件路径
TOKEN_CACHE_FILE = os.path.join(os.path.dirname(__file__), ".token_cache.json")
# Token 刷新间隔（秒），默认1小时
TOKEN_REFRESH_INTERVAL = 3600

def get_public_key_from_string(public_key_string: str):
    """
    从字符串中获取公钥对象
    
    Args:
        public_key_string: PEM 格式的公钥字符串
        
    Returns:
        PublicKey 对象
    """
    try:
        # 如果公钥字符串不包含头部和尾部，添加它们
        if not public_key_string.startswith("-----BEGIN"):
            public_key_string = f"-----BEGIN PUBLIC KEY-----\n{public_key_string}\n-----END PUBLIC KEY-----"
        
        # 从 PEM 格式字符串加载公钥
        public_key = serialization.load_pem_public_key(
            public_key_string.encode('utf-8'),
            backend=default_backend()
        )
        return public_key
    except Exception as e:
        raise Exception(f"加载公钥失败: {e}")


def encrypt(data: str, public_key_string: str) -> str:
    """
    RSA 公钥加密
    
    Args:
        data: 要加密的数据
        public_key_string: PEM 格式的公钥字符串
        
    Returns:
        Base64 编码的加密结果
    """
    try:
        public_key = get_public_key_from_string(public_key_string)
        
        # 使用 PKCS1v15 填充进行加密（对应 Java 的 "RSA"）
        encrypted = public_key.encrypt(
            data.encode('utf-8'),
            padding.PKCS1v15()
        )
        
        # Base64 编码
        return base64.b64encode(encrypted).decode('utf-8')
    except Exception as e:
        raise Exception(f"加密失败: {e}")


def save_token_to_cache(token: str):
    """
    将 token 保存到缓存文件
    
    Args:
        token: Authorization Token
    """
    try:
        cache_data = {
            "token": token,
            "timestamp": time.time()
        }
        with open(TOKEN_CACHE_FILE, 'w', encoding='utf-8') as f:
            json.dump(cache_data, f)
        print(f"✅ Token 已保存到缓存: {TOKEN_CACHE_FILE}")
    except Exception as e:
        print(f"⚠️ 保存 Token 到缓存失败: {e}")


def load_token_from_cache() -> Optional[str]:
    """
    从缓存文件加载 token
    
    Returns:
        Authorization Token，如果缓存不存在或已过期返回 None
    """
    try:
        if not os.path.exists(TOKEN_CACHE_FILE):
            return None
        
        with open(TOKEN_CACHE_FILE, 'r', encoding='utf-8') as f:
            cache_data = json.load(f)
        
        token = cache_data.get("token")
        timestamp = cache_data.get("timestamp", 0)
        
        # 检查 token 是否过期（超过1小时）
        if time.time() - timestamp > TOKEN_REFRESH_INTERVAL:
            print("⚠️ 缓存中的 Token 已过期")
            return None
        
        return token
    except Exception as e:
        print(f"⚠️ 从缓存加载 Token 失败: {e}")
        return None


def login(username: str, password: str, public_key: str) -> Optional[str]:
    """
    登录并获取 Authorization Token
    
    Args:
        username: 用户名
        password: 原始密码
        public_key: RSA 公钥字符串
        api_url: 登录 API 地址
        
    Returns:
        Authorization Token，如果失败返回 None
    """
    try:
        # 登录请求地址
        api_url = os.getenv("AI_CHAT_URL") + os.getenv("AI_CHAT_LOGIN_URI")

        # 加密密码
        encrypted_password = encrypt(password, public_key)
        
        # 准备请求数据
        payload = {
            "username": username,
            "password": encrypted_password
        }
        
        # 发送 POST 请求
        response = requests.post(
            api_url,
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=10
        )
        
        # 检查响应状态码
        if response.status_code == 200:
            result = response.json()
            # 从 data 字段中获取 token
            if "data" in result:
                data = result["data"]
                # 通常 token 可能在 data 的直接字段中，或者需要进一步解析
                # 根据实际 API 响应结构调整
                token = None
                if isinstance(data, str):
                    token = data
                elif isinstance(data, dict):
                    # 尝试常见的 token 字段名
                    for key in ["token", "access_token", "authorization", "auth_token"]:
                        if key in data:
                            token = data[key]
                            break
                    # 如果没有找到，返回整个 data（可能需要进一步处理）
                    if not token:
                        token = json.dumps(data)
                else:
                    token = str(data)
                
                if token:
                    # 保存到缓存
                    save_token_to_cache(token)
                    return token
                else:
                    print(f"❌ 无法从响应中提取 token: {result}")
                    return None
            else:
                print(f"❌ 响应中没有 data 字段: {result}")
                return None
        else:
            print(f"❌ 登录失败，状态码: {response.status_code}")
            print(f"   响应内容: {response.text}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"❌ 请求异常: {e}")
        return None
    except Exception as e:
        print(f"❌ 登录过程异常: {e}")
        return None


def refresh_token_task(username: str, password: str, public_key: str):
    """
    定时刷新 token 的后台任务
    
    Args:
        username: 用户名
        password: 原始密码
        public_key: RSA 公钥字符串
    """
    print(f"🔄 Token 刷新任务已启动，每 {TOKEN_REFRESH_INTERVAL // 60} 分钟刷新一次")
    
    while True:
        try:
            time.sleep(TOKEN_REFRESH_INTERVAL)
            print(f"🔄 开始刷新 Token...")
            token = login(username, password, public_key)
            if token:
                print(f"✅ Token 刷新成功")
            else:
                print(f"❌ Token 刷新失败，将在下次定时任务时重试")
        except Exception as e:
            print(f"❌ Token 刷新任务异常: {e}")
            time.sleep(60)  # 出错后等待1分钟再重试


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='获取 FZ Token')
    parser.add_argument('--username', type=str, help='用户名（如果未提供，从环境变量获取）')
    parser.add_argument('--password', type=str, help='原始密码（如果未提供，从环境变量获取）')
    parser.add_argument('--public-key', type=str, help='RSA 公钥字符串（PEM 格式，如果未提供，从环境变量获取）')
    parser.add_argument('--api-url', type=str, help='登录 API 地址（如果未提供，从环境变量获取）')
    parser.add_argument('--output', type=str, help='将 token 保存到文件（可选）')
    parser.add_argument('--daemon', action='store_true', help='以守护进程模式运行，定期刷新 token')
    parser.add_argument('--refresh-interval', type=int, default=3600, 
                       help='Token 刷新间隔（秒，默认: 3600，即1小时）')
    
    args = parser.parse_args()
    
    # 从环境变量或参数获取配置
    username = args.username or os.getenv("FZ_USERNAME")
    password = args.password or os.getenv("FZ_PASSWORD")
    public_key = args.public_key or os.getenv("PUBLIC_KEY")
    
    if not username or not password or not public_key:
        print("❌ 错误: 必须提供 username、password 和 public_key")
        print("   可以通过命令行参数或环境变量提供")
        return 1
    
    global TOKEN_REFRESH_INTERVAL
    if args.refresh_interval:
        TOKEN_REFRESH_INTERVAL = args.refresh_interval
    
    print("=" * 60)
    print("🔐 FZ Token 获取工具")
    print("=" * 60)
    print(f"用户名: {username}")
    if args.api_url:
        print(f"API 地址: {args.api_url}")
    print()
    
    # 执行登录
    token = login(username, password, public_key)
    
    if token:
        print("✅ 登录成功！")
        print(f"Authorization Token: {token[:50]}...")  # 只显示前50个字符
        
        # 如果指定了输出文件，保存 token
        if args.output:
            try:
                with open(args.output, 'w', encoding='utf-8') as f:
                    f.write(token)
                print(f"✅ Token 已保存到: {args.output}")
            except Exception as e:
                print(f"⚠️ 保存 Token 到文件失败: {e}")
        
        # 如果以守护进程模式运行，启动定时刷新任务
        if args.daemon:
            print(f"\n🔄 启动定时刷新任务（每 {TOKEN_REFRESH_INTERVAL // 60} 分钟刷新一次）...")
            refresh_thread = threading.Thread(
                target=refresh_token_task,
                args=(username, password, public_key),
                daemon=True
            )
            refresh_thread.start()
            
            print("✅ 守护进程模式已启动，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n🛑 停止守护进程")
        
        return 0
    else:
        print("❌ 登录失败")
        return 1


if __name__ == "__main__":
    sys.exit(main())

