# LLM Providers Guide

本指南介绍如何在 OM1 中配置和使用不同的 LLM（大语言模型）提供商。

## 支持的 LLM 提供商

OM1 目前支持以下 LLM 提供商：

| 提供商 | 类名 | SDK | 状态 |
|--------|------|-----|------|
| OpenAI | `OpenAILLM` | `openai` | ✅ 已支持 |
| DeepSeek | `DeepSeekLLM` | `openai` (兼容) | ✅ 已支持 |
| Gemini | `GeminiLLM` | `google-generativeai` | ✅ 已支持 |
| xAI (Grok) | `XaiLLM` | `openai` (兼容) | ✅ 已支持 |
| **Mistral AI** | `MistralLLM` | `mistralai` | 🆕 **新增** |
| **Meta Llama** | `LlamaLLM` | `llama-api-python` | 🆕 **新增** |

## 快速开始

### 1. 安装依赖

新的 LLM 提供商依赖已包含在 `pyproject.toml` 中：

```bash
uv sync  # 或者 pip install -e .
```

### 2. 配置环境变量

创建或更新 `.env` 文件：

```bash
# .env
OPENAI_API_KEY=sk-your-openai-key
MISTRAL_API_KEY=your-mistral-api-key
LLAMA_API_KEY=your-llama-api-key
DEEPSEEK_API_KEY=your-deepseek-key
```

### 3. 配置 JSON5 文件

在你的配置文件中（如 `config/spot.json5`），更新 `cortex_llm` 部分：

#### Mistral AI 配置
```json5
{
  "cortex_llm": {
    "type": "MistralLLM",
    "config": {
      "api_key": "${MISTRAL_API_KEY}",  // 从环境变量读取
      "model": "mistral-large-latest",   // 推荐模型
      "agent_name": "Spot",
      "history_length": 3,
      "timeout": 30
    }
  }
}
```

#### Meta Llama 配置
```json5
{
  "cortex_llm": {
    "type": "LlamaLLM", 
    "config": {
      "api_key": "${LLAMA_API_KEY}",
      "model": "llama-3.2-3b-instruct",  // 推荐模型
      "agent_name": "Spot",
      "history_length": 3,
      "timeout": 30
    }
  }
}
```

## 详细配置选项

### Mistral AI

#### 推荐模型
- `mistral-large-latest` - 最新的大模型（推荐）
- `mistral-medium` - 中等大小，平衡性能和成本
- `mistral-small` - 小模型，快速响应
- `open-mistral-7b` - 开源模型

#### 完整配置示例
```json5
{
  "cortex_llm": {
    "type": "MistralLLM",
    "config": {
      "api_key": "${MISTRAL_API_KEY}",
      "model": "mistral-large-latest",
      "base_url": "https://api.mistral.ai",  // 可选，默认官方 API
      "agent_name": "Assistant",
      "history_length": 5,
      "timeout": 30,
      "extra_params": {
        "temperature": 0.7,
        "max_tokens": 4096
      }
    }
  }
}
```

### Meta Llama

#### 推荐模型
- `llama-3.2-3b-instruct` - 轻量级指令模型（推荐）
- `llama-3.2-1b` - 超轻量级模型
- `llama-3.1-8b` - 中等大小模型
- `llama-3.1-70b` - 大模型（需要更多资源）

#### 完整配置示例
```json5
{
  "cortex_llm": {
    "type": "LlamaLLM",
    "config": {
      "api_key": "${LLAMA_API_KEY}",
      "model": "llama-3.2-3b-instruct",
      "agent_name": "Assistant",
      "history_length": 5,
      "timeout": 30,
      "extra_params": {
        "temperature": 0.8,
        "max_tokens": 2048
      }
    }
  }
}
```

## 本地/自托管部署

### 使用 Ollama 本地运行

Ollama 支持 OpenAI 兼容的 API 接口：

```json5
{
  "cortex_llm": {
    "type": "MistralLLM",  // 或者使用 OpenAILLM
    "config": {
      "base_url": "http://localhost:11434/v1",
      "api_key": "dummy-key",  // Ollama 不需要真实 API key
      "model": "mistral:latest",
      "timeout": 60  // 本地推理可能需要更长时间
    }
  }
}
```

### 使用 vLLM 服务器

```json5
{
  "cortex_llm": {
    "type": "LlamaLLM",
    "config": {
      "base_url": "http://localhost:8000",
      "api_key": "dummy-key",
      "model": "meta-llama/Llama-3.2-3B-Instruct",
      "timeout": 60
    }
  }
}
```

### 使用 LM Studio

```json5
{
  "cortex_llm": {
    "type": "OpenAILLM",  // LM Studio 提供 OpenAI 兼容接口
    "config": {
      "base_url": "http://localhost:1234/v1",
      "api_key": "lm-studio",
      "model": "your-local-model-name"
    }
  }
}
```

## 获取 API Keys

### Mistral AI
1. 访问 [console.mistral.ai](https://console.mistral.ai/)
2. 注册账户
3. 进入 API Keys 页面
4. 创建新的 API Key
5. 复制到环境变量：`MISTRAL_API_KEY=your-key-here`

### Meta Llama
1. 访问 [llama-api.com](https://llama-api.com/) 或相关服务
2. 注册账户并获取 API Key
3. 设置环境变量：`LLAMA_API_KEY=your-key-here`

**注意**：Llama 模型也可以通过其他服务获得，如：
- Together AI
- Replicate
- Hugging Face Inference API
- 本地部署（Ollama、vLLM、LM Studio）

## 测试配置

### 运行演示脚本

```bash
# 设置环境变量后运行
python demo_llm_providers.py
```

演示脚本会测试：
- 文本生成
- 对话功能
- 代码生成
- 提供商切换

### 运行测试套件

```bash
# 测试新的 LLM 插件
python -m pytest tests/llm/plugins/test_mistral_llm.py -v
python -m pytest tests/llm/plugins/test_llama_llm.py -v

# 测试所有 LLM 插件
python -m pytest tests/llm/plugins/ -v
```

## 故障排除

### 常见问题

**1. ImportError: No module named 'mistralai'**
```bash
# 重新安装依赖
uv sync
# 或者
pip install mistralai
```

**2. API Key 未找到**
- 检查 `.env` 文件是否正确设置
- 确认环境变量名称与配置文件中的引用一致
- 重启终端会话以加载新的环境变量

**3. 连接超时**
- 增加 `timeout` 配置值
- 检查网络连接
- 对于本地部署，确认服务器正在运行

**4. JSON 解析错误**
- Llama 模型有时需要额外的提示才能生成有效的 JSON
- 检查模型是否支持结构化输出
- 尝试不同的 `temperature` 设置

### 调试模式

启用调试日志：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### 性能优化

**Mistral AI:**
- 使用 `mistral-small` 获得更快响应
- 调整 `max_tokens` 限制输出长度
- 使用适当的 `temperature` 值（0.0-1.0）

**Meta Llama:**
- 选择适合任务的模型大小
- 本地部署可以获得更好的隐私保护
- 批处理请求可以提高效率

## 配置模板

### Spot 机器人配置（Mistral）

```json5
{
  "hertz": 1,
  "name": "spot_mistral",
  "system_prompt_base": "You are Spot, a smart and friendly dog robot...",
  
  "cortex_llm": {
    "type": "MistralLLM",
    "config": {
      "api_key": "${MISTRAL_API_KEY}",
      "model": "mistral-large-latest",
      "agent_name": "Spot",
      "history_length": 3,
      "timeout": 30
    }
  },
  
  "agent_inputs": [...],
  "agent_actions": [...]
}
```

### 对话助手配置（Llama）

```json5
{
  "hertz": 0.5,
  "name": "conversation_llama",
  "system_prompt_base": "You are a helpful AI assistant...",
  
  "cortex_llm": {
    "type": "LlamaLLM", 
    "config": {
      "api_key": "${LLAMA_API_KEY}",
      "model": "llama-3.2-3b-instruct",
      "agent_name": "Assistant",
      "history_length": 10,
      "extra_params": {
        "temperature": 0.7,
        "max_tokens": 2048
      }
    }
  },
  
  "agent_inputs": [...],
  "agent_actions": [...]
}
```

## 更多资源

- [OpenMind 文档](https://docs.openmind.org/)
- [Mistral AI 文档](https://docs.mistral.ai/)
- [Meta Llama 文档](https://llama.meta.com/docs/)
- [项目 GitHub](https://github.com/openmind/OM1)