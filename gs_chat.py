#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 语音对话系统（使用 G1 自带 ASR + 自有对话服务）

流程:
  G1 自带 ASR 识别  →  调用 https://ai-chat.zhinengjianshe.com/api/v1/conversation/stream
                    →  解析返回的 content 中的 {"text": "..."} 文本
                    →  使用 G1 的 TTS 播放回复

本文件基于 asr_subscriber_example.py，仅替换大模型调用逻辑。
"""

import json
import time
import requests

from G1.voice.example.asr_subscriber_example import (
    G1ChatWithASR,
    UNITREE_SDK_AVAILABLE,
    DEFAULT_API_KEY,  # 只是为了兼容原构造函数签名，不实际使用
)

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
except ImportError:
    AudioClient = None


GS_CHAT_URL = "https://ai-chat.zhinengjianshe.com/api/v1/conversation/stream"
GS_AUTH_TOKEN = "457179c2108e4448922ee7c06bbaf59f"


class G1ChatWithASR_GS(G1ChatWithASR):
    """
    覆盖原来的 DashScope 调用，改为调用公司的会话接口。
    """

    def call_dashscope_llm(self, text: str):
        """
        兼容原接口名，内部调用自有 HTTP 接口。

        只替换 "text" 字段，其它参数按用户要求写死：
          - clientType: "web"
          - content.type: "text"
          - contentType: "text"
          - conversationId: 固定 UUID
          - messageType: "internet_search"
          - modelName: "doubao"
          - modelType: "mcp"
          - moduleId: 固定值
          - webSearch: true
          - 其余字段按给定值固定
        """
        headers = {
            "Authorization": GS_AUTH_TOKEN,
            "Content-Type": "application/json;charset=utf-8",
            "Accept": "text/event-stream",
        }

        payload = {
            "chooseType": "",
            "clientType": "web",
            "content": {
                "type": "text",
                "text": text,  # 根据问题替换
            },
            "contentType": "text",
            "conversationId": "bb64e002-1c2d-4848-b7a7-597d79178f91",
            "deep": "0",
            "fileIds": "",
            "messageType": "internet_search",
            "modelName": "doubao",
            "modelType": "mcp",
            "moduleId": "b6512eca74e3370ebf7e26138cae1490",
            "repositoryIds": "",
            "webSearch": True,
            "zsk": "0",
        }

        self.get_logger().info("🌐 调用自有对话接口获取回复...")

        try:
            # 使用流式 SSE 读取
            with requests.post(
                GS_CHAT_URL,
                headers=headers,
                data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                stream=True,
                timeout=30,
            ) as resp:
                if resp.status_code != 200:
                    self.get_logger().error(
                        f"⚠️ 对话接口返回错误状态码: {resp.status_code}"
                    )
                    try:
                        self.get_logger().error(f"响应内容: {resp.text}")
                    except Exception:
                        pass
                    return None

                reply_text = ""

                for raw_line in resp.iter_lines(decode_unicode=True):
                    if not raw_line:
                        continue

                    line = raw_line.strip()
                    # 兼容 SSE: 行通常以 "data: " 开头
                    if line.startswith("data:"):
                        line = line[len("data:") :].strip()

                    try:
                        data = json.loads(line)
                    except Exception:
                        # 非 JSON 行，跳过
                        continue

                    # 期望结构类似 MessageRsp：
                    # {
                    #   "contentType": "text",
                    #   "content": "{\"text\":\"...\"}",
                    #   ...
                    # }
                    content_type = data.get("contentType")
                    if content_type != "text":
                        continue

                    content_raw = data.get("content")
                    if not content_raw:
                        continue

                    # content 是一个 JSON 字符串，内部有 {"text": "..."}
                    try:
                        inner = json.loads(content_raw)
                        text_value = inner.get("text", "")
                    except Exception:
                        # content 不是 JSON 字符串，尝试直接使用
                        text_value = content_raw

                    if text_value:
                        reply_text = text_value  # 不断覆盖，最后一条为最终回复

                reply_text = (reply_text or "").strip()
                if reply_text:
                    self.get_logger().info(f"💬 自有对话服务回复: {reply_text}")
                    return reply_text
                else:
                    self.get_logger().warn("⚠️ 对话接口未返回有效文本内容")
                    return None

        except requests.exceptions.Timeout:
            self.get_logger().error("⚠️ 对话接口请求超时")
            return None
        except Exception as e:
            self.get_logger().error(f"⚠️ 对话接口调用异常: {e}")
            return None


def main():
    """
    与 asr_subscriber_example.main 基本一致，只是使用 G1ChatWithASR_GS 节点。
    """
    import argparse
    import rclpy

    parser = argparse.ArgumentParser(
        description="G1 语音对话系统（使用G1自带ASR + 自有对话接口）"
    )
    parser.add_argument(
        "--network-interface",
        type=str,
        default=None,
        help="网络接口名称（用于连接G1，如 enp3s0）",
    )
    parser.add_argument(
        "--g1-ip",
        type=str,
        default=None,
        help="G1 的 IP 地址（当前未使用，仅保留接口）",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("G1 语音对话系统（使用G1自带ASR + 自有对话接口）")
    print("=" * 60)
    print("流程: G1自带ASR识别 → 自有对话接口 → G1 TTS播放\n")

    # 初始化 ROS2
    rclpy.init()

    # 初始化 G1 的 AudioClient（用于 TTS）
    tts_client = None
    if UNITREE_SDK_AVAILABLE and AudioClient is not None:
        try:
            if args.network_interface:
                ChannelFactoryInitialize(0, args.network_interface)
                print(f"✅ 使用网络接口: {args.network_interface}")
            else:
                ChannelFactoryInitialize(0)
                print("✅ 使用默认网络接口")

            tts_client = AudioClient()
            ret = tts_client.Init()
            if ret is not None and ret != 0:
                print(f"⚠️ G1 AudioClient初始化失败，代码: {ret}（将无法播放TTS）\n")
                tts_client = None
            else:
                # 尝试设置音量为 100% 激活 TTS
                try:
                    tts_client.SetVolume(100)
                    time.sleep(0.5)
                except Exception:
                    pass
        except Exception as e:
            print(f"⚠️ G1 TTS初始化异常: {e}（将无法播放TTS）\n")
            tts_client = None
    else:
        print("⚠️ unitree_sdk2py 未安装或 AudioClient 不可用，将无法播放TTS\n")

    # 创建使用自有对话接口的节点
    chat_node = G1ChatWithASR_GS(tts_client=tts_client, api_key=DEFAULT_API_KEY)

    print("=" * 60)
    print("📺 G1语音对话系统已启动（自有对话接口版，唤醒词模式）")
    print("=" * 60)
    print("说明:")
    print("  - 使用G1自带的语音识别（识别率较高）")
    print("  - 唤醒后：问题发送到自有对话接口处理，再用 G1 TTS 播放")
    print("  - 唤醒模式下说唤醒词: 回复'在的'")
    print("  - 回复过程中听到唤醒词: 逻辑打断当前回复，排队播放'在的'")
    print("按 Ctrl+C 退出程序\n")

    try:
        rclpy.spin(chat_node)
    except KeyboardInterrupt:
        print("\n\n🛑 停止服务")
    finally:
        chat_node.destroy_node()
        rclpy.shutdown()
        print("✅ 资源清理完成")


if __name__ == "__main__":
    main()


