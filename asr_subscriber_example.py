#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1语音对话系统（使用G1自带ASR）
G1自带ASR识别 → 阿里云大模型 → G1 TTS播放
"""
import sys
import time
import json
import requests
import threading
import subprocess
import os

# ROS2 Python接口
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    print("⚠️ rclpy未安装，请安装: sudo apt install ros-foxy-rclpy")
    ROS2_AVAILABLE = False
    sys.exit(1)

# Unitree SDK Python接口
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    # G1使用AudioClient（根据C++示例 unitree::robot::g1::AudioClient）
    try:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        UNITREE_SDK_AVAILABLE = True
        USE_AUDIO_CLIENT = True
    except ImportError:
        try:
            from unitree_sdk2py.go2.vui.g1_audio_client import AudioClient
            UNITREE_SDK_AVAILABLE = True
            USE_AUDIO_CLIENT = True
        except ImportError:
            UNITREE_SDK_AVAILABLE = False
            USE_AUDIO_CLIENT = False
except ImportError:
    print("⚠️ unitree_sdk2py未安装")
    UNITREE_SDK_AVAILABLE = False
    USE_AUDIO_CLIENT = False

# 阿里云DashScope配置
DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_API_KEY = "sk-ee503bb6532446b79a2c9bac1a6513c7"

class G1ChatWithASR(Node):
    """使用G1自带ASR的语音对话系统"""
    
    def __init__(self, tts_client=None, api_key=None):
        super().__init__('g1_chat_asr')
        
        self.tts_client = tts_client
        self.api_key = api_key or DEFAULT_API_KEY
        
        # 对话历史
        self.conversation_history = []
        
        # 防止重复处理同一句话
        self.last_processed_text = ""
        self.last_processed_time = 0
        self.processing_lock = threading.Lock()
        
        # 等待更好结果的机制（用于处理is_final=False的情况）
        self.pending_results = {}  # {text: (confidence, timestamp)}
        self.result_wait_time = 0.8  # 等待0.8秒看是否有更好的结果
        
        # 唤醒词功能（包含单独词和"同学"组合）
        # 注意：G1的ASR可能将"飞搏"识别为"飞博"、"飞波"等，所以包含多个变体
        self.wake_words = [
            "安全", "按全", "安泉" # 部分匹配（可能识别不完整）
        ]
        self.is_awakened = False  # 是否已唤醒
        self.last_interaction_time = 0  # 最后一次交互时间
        self.wake_timeout = 10.0  # 唤醒后10秒无交互则退出
        self.is_playing_tts = False  # 是否正在播放TTS
        self.current_tts_thread = None  # 当前TTS播放线程
        self.wake_reply = "你好，有什么可以帮到你的"
        self.wake_ack_reply = "在的"
        self.pending_wake_word = False  # 是否有待处理的唤醒词（用于打断）
        self.tts_lock = threading.Lock()  # TTS播放锁，防止并发播放
        
        # 订阅 audio_msg 话题（注意：不是rt/audio_msg）
        self.subscription = self.create_subscription(
            String,
            'audio_msg',
            self.asr_callback,
            10
        )
        
        self.get_logger().info('✅ 已订阅话题: audio_msg')
        self.get_logger().info(f'🎯 唤醒词: {", ".join(self.wake_words[:3])}...')
        self.get_logger().info('💡 等待唤醒词或语音识别结果...\n')
        
        # 启动超时检测线程
        self.timeout_thread = threading.Thread(target=self._timeout_checker, daemon=True)
        self.timeout_thread.start()
    
    def asr_callback(self, msg):
        """收到ASR识别结果时的回调函数"""
        # 消息是JSON格式，需要解析
        try:
            data = json.loads(msg.data)
            recognized_text = data.get('text', '').strip()
            is_final = data.get('is_final', False)
            confidence = data.get('confidence', 0.0)
            
            # 调试：打印收到的原始消息（如果文本为空也打印，帮助排查问题）
            if not recognized_text:
                self.get_logger().debug(f'📥 收到ASR消息但文本为空: is_final={is_final}, confidence={confidence:.2f}')
                return
            
            # 调试：打印收到的识别结果
            self.get_logger().debug(f'📥 收到ASR消息: "{recognized_text}" (is_final={is_final}, confidence={confidence:.2f})')
            
            current_time = time.time()
            
            # 如果是最终结果，直接处理（优先级最高）
            if is_final:
                # 清除该文本的待处理记录
                if recognized_text in self.pending_results:
                    del self.pending_results[recognized_text]
                
                # 防止重复处理（但唤醒词允许连续唤醒）
                has_wake_word = self._check_wake_word(recognized_text)
                if not has_wake_word:  # 非唤醒词才检查重复
                    if recognized_text == self.last_processed_text and (current_time - self.last_processed_time) < 2.0:
                        self.get_logger().debug(f'⏭️  跳过重复文本: {recognized_text}')
                        return
                
                # 未唤醒时，只检查唤醒词，不显示其他识别结果
                if not self.is_awakened:
                    if has_wake_word:
                        self.get_logger().info(f'🎤 识别结果（最终）: {recognized_text} (置信度: {confidence:.2f})')
                        self._process_recognized_text(recognized_text, current_time)
                    else:
                        # 未唤醒且没有唤醒词，不显示也不处理
                        pass
                else:
                    # 已唤醒，显示并处理
                    self.get_logger().info(f'🎤 识别结果（最终）: {recognized_text} (置信度: {confidence:.2f})')
                    self._process_recognized_text(recognized_text, current_time)
                return
            
            # 对于非最终结果，根据置信度决定
            # G1的ASR可能一直返回is_final=False，所以需要智能处理
            # 关键优化：先检查唤醒词，唤醒词降低置信度要求
            has_wake_word = self._check_wake_word(recognized_text)
            # 唤醒词降低置信度阈值（0.3），普通文本保持0.5
            confidence_threshold = 0.3 if has_wake_word else 0.5
            
            if confidence >= confidence_threshold:
                # 关键优化：如果检测到唤醒词，无论是否正在播放，都要立即处理
                if has_wake_word:
                    # 如果已唤醒，直接回复"在的"（无论是否正在播放）
                    if self.is_awakened:
                        self.get_logger().info('🎯 唤醒模式下检测到唤醒词，立即回复"在的"')
                        # 立即设置打断标志（如果正在播放）
                        if self.is_playing_tts:
                            self.pending_wake_word = True
                            self.is_playing_tts = False  # 停止等待循环
                        self.last_interaction_time = current_time
                        
                        # 尝试PlayStop("voice") - 可能对系统TTS有效
                        try:
                            if self.tts_client:
                                try:
                                    self.tts_client.PlayStop("voice")
                                    self.get_logger().debug('📝 已调用PlayStop("voice")')
                                    time.sleep(0.05)
                                except Exception as e:
                                    self.get_logger().debug(f'📝 PlayStop异常: {e}')
                                
                                # 发送"在的"回复（会排队）
                                ret = self.tts_client.TtsMaker(self.wake_ack_reply, 0)
                                if isinstance(ret, tuple):
                                    ret_code = ret[0]
                                    if ret_code == 0:
                                        self.get_logger().info('✅ "在的"回复已发送（会排队播放）')
                                    else:
                                        self.get_logger().warn(f'⚠️ 回复返回码: {ret_code}')
                                elif ret == 0 or ret is None:
                                    self.get_logger().info('✅ "在的"回复已发送（会排队播放）')
                                else:
                                    self.get_logger().warn(f'⚠️ 回复返回值: {ret}')
                        except Exception as e:
                            self.get_logger().error(f'❌ 发送"在的"回复异常: {e}')
                        
                        # 关键：发送"在的"后，立即重置pending_wake_word，允许后续TTS播放
                        self.pending_wake_word = False
                        
                        # 更新处理记录（允许连续唤醒）
                        self.last_processed_text = recognized_text
                        self.last_processed_time = current_time
                        return
                    else:
                        # 未唤醒，立即处理唤醒词（唤醒系统）
                        self.get_logger().info(f'🎯 检测到唤醒词（中间结果），立即处理: {recognized_text} (置信度: {confidence:.2f})')
                        self._process_recognized_text(recognized_text, current_time)
                        return
                
                # 非唤醒词：记录到待处理列表
                # 如果已经有相同文本但置信度更低，更新它
                if recognized_text in self.pending_results:
                    old_confidence, old_time = self.pending_results[recognized_text]
                    if confidence > old_confidence:
                        # 新结果置信度更高，更新
                        self.pending_results[recognized_text] = (confidence, current_time)
                        self.get_logger().info(f'🔄 更新识别结果: {recognized_text} (置信度: {old_confidence:.2f} → {confidence:.2f})')
                    else:
                        # 新结果置信度更低，忽略
                        return
                else:
                    # 新文本，添加到待处理列表
                    self.pending_results[recognized_text] = (confidence, current_time)
                    # 已唤醒时，显示识别结果
                    if self.is_awakened:
                        self.get_logger().info(f'🎤 识别结果（中间）: {recognized_text} (置信度: {confidence:.2f}, 等待更好结果...)')
                
                # 启动延迟处理线程（等待看是否有更好的结果）
                threading.Thread(
                    target=self._delayed_process,
                    args=(recognized_text, confidence, current_time),
                    daemon=True
                ).start()
            else:
                # 置信度<0.5，直接忽略
                self.get_logger().debug(f'⏭️  跳过低置信度结果: {recognized_text} (置信度: {confidence:.2f})')
            
        except json.JSONDecodeError:
            # 如果不是JSON格式，直接使用原始数据
            recognized_text = msg.data.strip()
            if recognized_text:
                current_time = time.time()
                # 未唤醒时，只处理唤醒词
                if not self.is_awakened:
                    if self._check_wake_word(recognized_text):
                        self.get_logger().info(f'🎤 识别结果: {recognized_text}')
                        self._process_recognized_text(recognized_text, current_time)
                else:
                    # 已唤醒，正常处理
                    self.get_logger().info(f'🎤 识别结果: {recognized_text}')
                    if recognized_text != self.last_processed_text or (current_time - self.last_processed_time) >= 2.0:
                        if not self.processing_lock.locked():
                            threading.Thread(
                                target=self.process_user_input,
                                args=(recognized_text,),
                                daemon=True
                            ).start()
                        self.last_processed_text = recognized_text
                        self.last_processed_time = current_time
                        self.last_interaction_time = current_time
        except Exception as e:
            self.get_logger().error(f'❌ 解析消息失败: {e}')
            return
    
    def _delayed_process(self, text, confidence, timestamp):
        """延迟处理：等待一段时间看是否有更好的结果"""
        time.sleep(self.result_wait_time)
        
        # 检查是否还在待处理列表中，且没有被更新
        if text in self.pending_results:
            current_confidence, current_timestamp = self.pending_results[text]
            # 如果置信度没有变化，说明没有更好的结果，可以处理了
            if current_confidence == confidence and current_timestamp == timestamp:
                # 从待处理列表中移除
                del self.pending_results[text]
                
                # 防止重复处理（但唤醒词允许连续唤醒）
                current_time = time.time()
                has_wake_word = self._check_wake_word(text)
                if not has_wake_word:  # 非唤醒词才检查重复
                    if text == self.last_processed_text and (current_time - self.last_processed_time) < 2.0:
                        self.get_logger().debug(f'⏭️  跳过重复文本: {text}')
                        return
                
                # 未唤醒时，只处理唤醒词
                if not self.is_awakened:
                    if has_wake_word:
                        self.get_logger().info(f'✅ 处理识别结果: {text} (置信度: {confidence:.2f}, 等待后无更好结果)')
                        self._process_recognized_text(text, current_time)
                    # 否则不处理，也不显示
                else:
                    # 已唤醒，正常处理
                    self.get_logger().info(f'✅ 处理识别结果: {text} (置信度: {confidence:.2f}, 等待后无更好结果)')
                    self._process_recognized_text(text, current_time)
    
    def _check_wake_word(self, text):
        """检查是否包含唤醒词（支持谐音和部分匹配）"""
        # 不转换为小写，保持原始大小写，因为中文没有大小写
        # 先检查完整匹配（优先级高）
        for wake_word in self.wake_words:
            if wake_word in text:
                self.get_logger().debug(f'🎯 检测到唤醒词: "{wake_word}" 在 "{text}" 中')
                return True
        
        # 如果完整匹配失败，尝试部分匹配（处理识别不完整的情况）
        # 检查是否包含"飞搏"、"飞博"、"飞波"等核心词
        core_words = ["飞搏", "飞博", "飞波", "飞薄", "飞伯"]
        for core_word in core_words:
            if core_word in text:
                # 进一步检查：如果文本很短（可能是单独词），或者包含"同"（可能是"同学"的一部分）
                if len(text) <= 4 or "同" in text:
                    self.get_logger().debug(f'🎯 检测到核心唤醒词: "{core_word}" 在 "{text}" 中（部分匹配）')
                    return True
        
        return False
    
    def _wait_for_new_tts(self, text_length):
        """等待新的TTS播放完成"""
        try:
            estimated_time = text_length / 3.5
            time.sleep(min(estimated_time, 2.0))
            self.is_playing_tts = False
            self.get_logger().info('✅ 打断后的TTS播放完成')
        except Exception as e:
            self.get_logger().error(f'❌ 等待TTS异常: {e}')
            self.is_playing_tts = False
    
    def _interrupt_and_reply(self, reply_text):
        """打断当前播放并立即回复（在单独线程中执行）"""
        # 根据官方文档，G1支持打断TTS播放
        # 方法：发送新的TTS命令来覆盖当前播放
        try:
            # 先尝试发送一个很短的命令来"清空"播放队列（可选）
            # 然后立即发送新的回复
            self.get_logger().info(f'🛑 打断后立即发送新TTS命令: {reply_text}')
            
            # 立即发送新命令（G1的TTS应该支持新命令覆盖旧命令）
            ret = self.tts_client.TtsMaker(reply_text, 0)
            if isinstance(ret, tuple):
                ret_code = ret[0]
                if ret_code == 0:
                    self.get_logger().info(f'✅ 打断TTS命令发送成功')
                    # 更新播放状态
                    self.is_playing_tts = True  # 标记新的播放开始
                    # 估算新播放时间
                    estimated_time = len(reply_text) / 3.5
                    # 等待新播放完成
                    time.sleep(min(estimated_time, 2.0))
                    self.is_playing_tts = False
                else:
                    self.get_logger().warn(f'⚠️ 打断TTS命令返回码: {ret_code}')
            elif ret == 0 or ret is None:
                self.get_logger().info(f'✅ 打断TTS命令发送成功')
                # 更新播放状态
                self.is_playing_tts = True
                estimated_time = len(reply_text) / 3.5
                time.sleep(min(estimated_time, 2.0))
                self.is_playing_tts = False
            else:
                self.get_logger().warn(f'⚠️ 打断TTS命令返回值: {ret}')
        except Exception as e:
            self.get_logger().error(f'❌ 打断TTS异常: {e}')
            import traceback
            traceback.print_exc()
    
    def _process_recognized_text(self, recognized_text, current_time):
        """处理识别到的文本"""
        # 检查是否包含唤醒词
        has_wake_word = self._check_wake_word(recognized_text)
        
        # 如果已唤醒且检测到唤醒词，回复"在的"（无论是否正在播放）
        if self.is_awakened and has_wake_word:
            # 允许连续唤醒，即使文本相同也处理
            self.get_logger().info('🎯 唤醒模式下检测到唤醒词，立即回复"在的"')
            # 如果正在播放，设置打断标志
            if self.is_playing_tts:
                self.pending_wake_word = True
                self.is_playing_tts = False  # 停止等待循环
            
            # 尝试PlayStop("voice") - 可能对系统TTS有效
            try:
                if self.tts_client:
                    try:
                        self.tts_client.PlayStop("voice")
                        self.get_logger().debug('📝 已调用PlayStop("voice")')
                        time.sleep(0.05)
                    except Exception as e:
                        self.get_logger().debug(f'📝 PlayStop异常: {e}')
                    
                    # 直接发送"在的"（会排队，但逻辑打断已生效）
                    ret = self.tts_client.TtsMaker(self.wake_ack_reply, 0)
                    if isinstance(ret, tuple):
                        ret_code = ret[0]
                        if ret_code == 0:
                            self.get_logger().info('✅ "在的"回复已发送（会排队播放）')
                        else:
                            self.get_logger().warn(f'⚠️ 回复返回码: {ret_code}')
                    elif ret == 0 or ret is None:
                        self.get_logger().info('✅ "在的"回复已发送（会排队播放）')
                    else:
                        self.get_logger().warn(f'⚠️ 回复返回值: {ret}')
            except Exception as e:
                self.get_logger().error(f'❌ 发送"在的"回复异常: {e}')
            
            # 关键：发送"在的"后，立即重置pending_wake_word，允许后续TTS播放
            self.pending_wake_word = False
            
            self.last_interaction_time = current_time
            # 更新处理记录（允许连续唤醒）
            self.last_processed_text = recognized_text
            self.last_processed_time = current_time
            return
        
        # 如果未唤醒，只检查唤醒词（允许连续唤醒，不检查重复）
        if not self.is_awakened:
            if has_wake_word:
                # 允许连续唤醒，即使文本相同也处理
                self.get_logger().info(f'🎯🎯🎯 检测到唤醒词，唤醒对话系统 🎯🎯🎯')
                self.is_awakened = True
                self.last_interaction_time = current_time
                # 播放唤醒回复
                self.play_tts(self.wake_reply)
                # 更新处理记录（但允许连续唤醒）
                self.last_processed_text = recognized_text
                self.last_processed_time = current_time
            else:
                # 未唤醒且没有唤醒词，不处理
                self.get_logger().debug(f'⏭️  未唤醒，忽略: {recognized_text}')
            return
        
        # 已唤醒状态，处理用户输入
        # 检查退出命令
        exit_keywords = ['退出', '结束', '再见', '拜拜']
        if any(keyword in recognized_text for keyword in exit_keywords):
            self.get_logger().info('👋 检测到退出命令，退出唤醒模式...')
            self.is_awakened = False
            self.conversation_history = []
            reply = "好的，退出唤醒模式"
            self.play_tts(reply)
            return
        
        # 使用线程处理，避免阻塞ROS2回调
        if not self.processing_lock.locked():
            threading.Thread(
                target=self.process_user_input,
                args=(recognized_text,),
                daemon=True
            ).start()
        
        # 更新最后处理的时间和文本
        self.last_processed_text = recognized_text
        self.last_processed_time = current_time
        self.last_interaction_time = current_time  # 更新交互时间
    
    def process_user_input(self, text):
        """处理用户输入：调用大模型并播放回复"""
        with self.processing_lock:
            try:
                # 检查是否仍然在唤醒状态
                if not self.is_awakened:
                    self.get_logger().debug('⏭️  已退出唤醒模式，忽略输入')
                    return
                
                self.get_logger().info(f'🤖 正在调用大模型...')
                
                # 调用大模型
                reply = self.call_dashscope_llm(text)
                
                if reply:
                    self.get_logger().info(f'💬 大模型回复: {reply}')
                    # 播放TTS（更新交互时间）
                    self.play_tts(reply)
                    self.last_interaction_time = time.time()
                else:
                    self.get_logger().warn('⚠️ 大模型未返回回复')
                    
            except Exception as e:
                self.get_logger().error(f'❌ 处理用户输入异常: {e}')
                import traceback
                traceback.print_exc()
    
    def _timeout_checker(self):
        """超时检测线程：10秒无交互则退出唤醒模式"""
        while True:
            time.sleep(1)  # 每秒检查一次
            if self.is_awakened:
                current_time = time.time()
                # 如果超过10秒没有交互，且不在播放TTS，退出唤醒模式
                if (current_time - self.last_interaction_time) > self.wake_timeout:
                    if not self.is_playing_tts:
                        self.get_logger().info('⏰ 超过10秒无交互，退出唤醒模式')
                        self.is_awakened = False
                        self.conversation_history = []
    
    def call_dashscope_llm(self, text):
        """调用阿里云DashScope大模型API"""
        try:
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            
            # 添加系统提示词
            messages = [
                {
                    "role": "system",
                    "content": "你是一个友好的AI助手，名字叫小G。请用简洁、友好的方式回答问题。"
                }
            ]
            
            # 添加对话历史（最近5轮）
            messages.extend(self.conversation_history[-10:])
            
            # 添加当前用户消息
            messages.append({"role": "user", "content": text})
            
            data = {
                "model": "qwen-turbo",  # 使用通义千问模型
                "messages": messages,
                "temperature": 0.7,
                "max_tokens": 500
            }
            
            # 发送请求
            response = requests.post(
                f"{DASHSCOPE_BASE_URL}/chat/completions",
                headers=headers,
                json=data,
                timeout=15
            )
            
            if response.status_code == 200:
                result = response.json()
                reply = result["choices"][0]["message"]["content"].strip()
                
                # 更新对话历史
                self.conversation_history.append({"role": "user", "content": text})
                self.conversation_history.append({"role": "assistant", "content": reply})
                
                # 限制历史长度（保留最近10条消息）
                if len(self.conversation_history) > 10:
                    self.conversation_history = self.conversation_history[-10:]
                
                return reply
            else:
                self.get_logger().error(f'⚠️ 大模型API错误: {response.status_code}')
                self.get_logger().error(f'   响应: {response.text}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'⚠️ 大模型调用异常: {e}')
            import traceback
            traceback.print_exc()
            return None
    
    
    def play_tts(self, text):
        """使用G1的TTS播放文本（支持打断）"""
        if not self.tts_client:
            self.get_logger().warn('⚠️ TTS客户端未初始化，无法播放')
            return False
        
        # 使用锁防止并发播放（但允许打断）
        with self.tts_lock:
            # 如果已经有待处理的唤醒词，不播放
            if self.pending_wake_word:
                self.get_logger().info('⏭️  检测到待处理的唤醒词，跳过当前播放')
                self.pending_wake_word = False
                return False
            
            # 标记正在播放TTS（在发送命令之前）
            self.is_playing_tts = True
            
            try:
                self.get_logger().info(f'📢 正在发送TTS命令到G1...')
                self.get_logger().info(f'📝 文本内容: {text}')
                
                # 确保TTS客户端仍然有效
                try:
                    # 先检查音量，确保连接正常
                    volume_check = self.tts_client.GetVolume()
                    if isinstance(volume_check, tuple):
                        _, vol_dict = volume_check
                        vol = vol_dict.get('volume', '未知')
                        self.get_logger().info(f'🔊 当前音量: {vol}%')
                except Exception as e:
                    self.get_logger().warn(f'⚠️ 无法获取音量: {e}')
                
                # 调用TTS
                ret = self.tts_client.TtsMaker(text, 0)
                
                # 处理返回值（与g1_chat.py保持一致）
                if isinstance(ret, tuple):
                    ret_code = ret[0]
                    self.get_logger().info(f'📢 TTS返回: {ret} (元组格式)')
                    if ret_code == 0:
                        # 估算播放时间（中文大约每秒3-4个字）
                        estimated_time = len(text) / 3.5
                        self.get_logger().info(f'⏳ 预计播放时间: {estimated_time:.1f}秒')
                        self.get_logger().info(f'✅ TTS命令发送成功')
                        
                        # 等待播放完成（可以被唤醒词打断）
                        sleep_interval = 0.02  # 每0.02秒检查一次是否被打断（非常频繁）
                        total_slept = 0
                        while total_slept < estimated_time and self.is_playing_tts:
                            time.sleep(sleep_interval)
                            total_slept += sleep_interval
                            # 检查是否有待处理的唤醒词或被打断
                            if self.pending_wake_word or not self.is_playing_tts:
                                self.get_logger().info('🛑 检测到打断信号，立即停止等待')
                                break
                        
                        self.is_playing_tts = False
                        return True
                    else:
                        self.get_logger().error(f'⚠️ TTS播放失败，代码: {ret_code}')
                        self.is_playing_tts = False
                        return False
                elif ret == 0 or ret is None:
                    self.get_logger().info(f'📢 TTS返回: {ret}')
                    # 估算播放时间
                    estimated_time = len(text) / 3.5
                    self.get_logger().info(f'⏳ 预计播放时间: {estimated_time:.1f}秒')
                    self.get_logger().info(f'✅ TTS命令发送成功')
                    
                    # 等待播放完成（可以被唤醒词打断）
                    sleep_interval = 0.02  # 每0.02秒检查一次（非常频繁）
                    total_slept = 0
                    while total_slept < estimated_time and self.is_playing_tts:
                        time.sleep(sleep_interval)
                        total_slept += sleep_interval
                        # 检查是否有待处理的唤醒词
                        if self.pending_wake_word or not self.is_playing_tts:
                            self.get_logger().info('🛑 检测到打断信号，立即停止等待')
                            break
                    
                    self.is_playing_tts = False
                    return True
                else:
                    self.get_logger().error(f'⚠️ TTS播放失败，代码: {ret}')
                    self.is_playing_tts = False
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'❌ TTS异常: {e}')
                import traceback
                traceback.print_exc()
                self.is_playing_tts = False
                
                # 尝试重新初始化TTS客户端（与g1_chat.py保持一致）
                self.get_logger().info('🔄 尝试重新初始化TTS客户端...')
                try:
                    self.tts_client.Init()
                    self.get_logger().info('✅ TTS客户端重新初始化成功')
                except:
                    self.get_logger().error('❌ TTS客户端重新初始化失败')
                
                return False

def disable_g1_builtin_voice(g1_ip=None, g1_user='unitree'):
    """
    尝试禁用G1自带的语音助手（笨笨同学）
    通过SSH连接到G1并停止相关服务
    """
    if not g1_ip:
        # 尝试自动检测G1 IP（通过ping常见IP）
        try:
            # 常见的G1 IP范围
            common_ips = ['192.168.123.15', '192.168.123.161', '192.168.1.100']
            for ip in common_ips:
                try:
                    result = subprocess.run(['ping', '-c', '1', '-W', '1', ip], 
                                          capture_output=True, timeout=2)
                    if result.returncode == 0:
                        g1_ip = ip
                        print(f"   📡 自动检测到G1 IP: {g1_ip}")
                        break
                except:
                    continue
        except:
            pass
    
    if not g1_ip:
        print("   ⚠️ 未指定G1 IP地址，无法通过SSH禁用")
        print("   💡 提示：可以通过 --g1-ip 参数指定G1的IP地址")
        return False
    
    try:
        print(f"   📡 连接到G1: {g1_user}@{g1_ip}")
        
        # 方法1: 尝试停止语音服务（通过systemctl）
        commands = [
            # 停止可能的语音服务
            "sudo systemctl stop unitree-voice 2>/dev/null || true",
            "sudo systemctl stop g1-voice 2>/dev/null || true",
            "sudo systemctl stop vui-service 2>/dev/null || true",
            # 停止可能的ROS2节点
            "pkill -f 'vui' 2>/dev/null || true",
            "pkill -f 'voice' 2>/dev/null || true",
            "pkill -f 'audio_msg' 2>/dev/null || true",
        ]
        
        # 执行SSH命令
        success_count = 0
        for cmd in commands:
            try:
                ssh_cmd = f'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 {g1_user}@{g1_ip} "{cmd}"'
                result = subprocess.run(ssh_cmd, shell=True, capture_output=True, timeout=5)
                if result.returncode == 0:
                    print(f"   ✅ 执行命令成功: {cmd.split()[0]}...")
                    success_count += 1
            except subprocess.TimeoutExpired:
                print(f"   ⚠️ 命令超时: {cmd.split()[0]}...")
            except Exception as e:
                print(f"   ⚠️ 执行命令失败: {cmd.split()[0]}... ({e})")
        
        if success_count > 0:
            return True
        else:
            print("   ⚠️ 所有命令都执行失败，可能需要手动关闭")
            return False
        
    except Exception as e:
        print(f"   ❌ SSH连接失败: {e}")
        print(f"   💡 提示：")
        print(f"      1. 确保G1和PC在同一网络")
        print(f"      2. 确保SSH已启用（G1默认用户名: unitree）")
        print(f"      3. 确保已配置SSH密钥或密码认证")
        print(f"      4. 或者手动在G1上关闭语音助手")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='G1语音对话系统（使用G1自带ASR）')
    parser.add_argument('--network-interface', type=str, default=None,
                       help='网络接口名称（用于连接G1，如enp3s0）')
    parser.add_argument('--api-key', type=str, default=DEFAULT_API_KEY,
                       help=f'阿里云DashScope API密钥（默认使用内置密钥）')
    parser.add_argument('--g1-ip', type=str, default=None,
                       help='G1的IP地址（用于SSH关闭自带语音服务，如192.168.123.15）')
    parser.add_argument('--g1-user', type=str, default='unitree',
                       help='G1的SSH用户名（默认: unitree）')
    parser.add_argument('--disable-builtin-voice', action='store_true',
                       help='禁用G1自带的语音助手（笨笨同学），避免与自定义唤醒词冲突（强烈推荐）')
    
    args = parser.parse_args()
    
    print("="*60)
    print("G1语音对话系统（使用G1自带ASR）")
    print("="*60)
    print("流程: G1自带ASR识别 → 阿里云大模型 → G1 TTS播放\n")
    
    # 如果需要禁用G1自带语音助手
    if args.disable_builtin_voice:
        print("🔧 尝试禁用G1自带的语音助手（笨笨同学）...")
        if disable_g1_builtin_voice(args.g1_ip, args.g1_user):
            print("✅ G1自带语音助手已禁用\n")
        else:
            print("⚠️ 无法禁用G1自带语音助手，请手动在G1上关闭\n")
    
    # 初始化ROS2
    rclpy.init()
    
    # 初始化G1的AudioClient（用于TTS）
    tts_client = None
    if UNITREE_SDK_AVAILABLE:
        try:
            if args.network_interface:
                ChannelFactoryInitialize(0, args.network_interface)
                print(f"✅ 使用网络接口: {args.network_interface}")
            else:
                ChannelFactoryInitialize(0)
                print("✅ 使用默认网络接口")
            
            if USE_AUDIO_CLIENT:
                # 使用AudioClient（G1，根据C++示例）
                tts_client = AudioClient()
                ret = tts_client.Init()
                
                # 某些版本的Init()可能返回None表示成功，或者返回0表示成功
                if ret is not None and ret != 0:
                    print(f"⚠️ G1 AudioClient初始化失败，代码: {ret}（将无法播放TTS）\n")
                    tts_client = None
                else:
                    # 尝试设置超时
                    try:
                        tts_client.SetTimeout(10.0)
                    except Exception:
                        pass  # 某些版本可能不支持SetTimeout
                    
                    # 通过实际调用验证初始化是否成功（与g1_audio_client_example.py保持一致）
                    try:
                        # 先获取音量验证连接（与g1_audio_client_example.py一致）
                        volume_result = tts_client.GetVolume()
                        if isinstance(volume_result, tuple) and len(volume_result) == 2:
                            ret_code, volume_dict = volume_result
                            current_volume = volume_dict.get('volume', '未知')
                            print(f"✅ G1 AudioClient初始化成功（当前音量: {current_volume}%）")
                            
                            # 关键：总是设置音量为100%（与g1_audio_client_example.py一致）
                            # 这可能是激活TTS服务所必需的步骤
                            print("   🔊 设置音量为100%（激活TTS服务）...")
                            try:
                                set_vol_ret = tts_client.SetVolume(100)
                                if set_vol_ret == 0:
                                    print("   ✅ 设置音量为100%")
                                    time.sleep(0.5)
                                    # 再次确认音量
                                    volume_result2 = tts_client.GetVolume()
                                    if isinstance(volume_result2, tuple) and len(volume_result2) == 2:
                                        _, volume_dict2 = volume_result2
                                        actual_volume = volume_dict2.get('volume', '未知')
                                        print(f"   ✅ 确认音量已设置为: {actual_volume}%")
                                else:
                                    print(f"   ⚠️ 设置音量失败，代码: {set_vol_ret}")
                            except Exception as e:
                                print(f"   ⚠️ 设置音量异常: {e}")
                                import traceback
                                traceback.print_exc()
                            
                            # 测试TTS（确保功能正常）
                            print("   🧪 测试TTS功能...")
                            print(f"   📝 测试文本: '测试'")
                            test_ret = tts_client.TtsMaker("测试", 0)
                            print(f"   📢 TTS返回: {test_ret}")
                            
                            # 详细检查返回值
                            if isinstance(test_ret, tuple):
                                ret_code = test_ret[0]
                                print(f"   📊 返回码: {ret_code}")
                                if ret_code == 0:
                                    print("   ✅ TTS命令发送成功（应该听到'测试'）")
                                    # 等待播放完成（参考g1_audio_client_example.py，等待5秒）
                                    print("   ⏳ 等待播放完成（约2秒）...")
                                    time.sleep(2)
                                    print("   ✅ TTS测试完成，请检查G1是否发出声音")
                                else:
                                    print(f"   ⚠️ TTS测试失败，返回码: {ret_code}")
                            elif test_ret == 0 or test_ret is None:
                                print("   ✅ TTS命令发送成功（应该听到'测试'）")
                                # 等待播放完成
                                print("   ⏳ 等待播放完成（约2秒）...")
                                time.sleep(2)
                                print("   ✅ TTS测试完成，请检查G1是否发出声音")
                            else:
                                print(f"   ⚠️ TTS测试失败，返回值: {test_ret}")
                            
                            print()
                        else:
                            print(f"✅ G1 AudioClient初始化成功\n")
                    except Exception as e:
                        print(f"⚠️ G1 AudioClient初始化可能失败（无法获取音量）: {e}（将无法播放TTS）\n")
                        import traceback
                        traceback.print_exc()
                        tts_client = None
            else:
                print("⚠️ AudioClient不可用，TTS功能将不可用\n")
                tts_client = None
        except Exception as e:
            print(f"⚠️ G1 TTS初始化异常: {e}（将无法播放TTS）\n")
            import traceback
            traceback.print_exc()
            tts_client = None
    else:
        print("⚠️ unitree_sdk2py未安装，将无法播放TTS\n")
    
    # 创建对话节点
    chat_node = G1ChatWithASR(tts_client=tts_client, api_key=args.api_key)
    
    print("="*60)
    print("📺 G1语音对话系统已启动（唤醒词模式）")
    print("="*60)
    print("说明:")
    print("  - 使用G1自带的语音识别（识别率较高）")
    print("  - 唤醒词: 飞搏/飞博/飞波（单独或加'同学'）")
    print("  - 建议: 使用 --disable-builtin-voice 参数禁用G1自带语音助手，避免冲突")
    print("  - 未唤醒时: 不显示识别结果，不调用大模型")
    print("  - 唤醒后: 自动回复欢迎语，开始对话")
    print("  - 唤醒模式下说唤醒词: 回复'在的'")
    print("  - 回复过程中听到唤醒词: 立即打断，回复'在的'")
    print("  - 10秒无交互: 自动退出唤醒模式")
    print("  - 说'退出'、'结束'或'再见': 手动退出唤醒模式")
    print("按 Ctrl+C 退出程序\n")
    
    try:
        # 自旋节点，等待消息
        rclpy.spin(chat_node)
    except KeyboardInterrupt:
        print("\n\n🛑 停止服务")
    finally:
        chat_node.destroy_node()
        rclpy.shutdown()
        print("✅ 资源清理完成")

if __name__ == "__main__":
    main()


