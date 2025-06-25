---

title: "Meet Mu: Microsoft’s On‑Device AI Powering Windows Settings"
excerpt: "Microsoft’s new micro-language model, Mu, delivers fast, local AI in Windows Settings, bringing seamless natural language control to system configuration with sub‑500 ms response times."
coverImage: "/assets/blog/Mu.png"
date: "2025-06-23T12:00:00.000000"
author:
  name: Microsoft AI Team
  picture: /assets/blog/authors/aura.png
ogImage:
  url: "/assets/blog/Mu.png"

---

# Meet Mu: Microsoft’s On‑Device AI Powering Windows Settings

Microsoft today introduced **Mu**, a purpose-built, **on‑device small language model** (SML) that powers the new AI Agent feature in Windows Settings. Designed to run entirely on Neural Processing Units (NPUs) in Copilot+ PCs, Mu brings advanced natural language control to system configuration with no compromises on speed or privacy.

## Why Mu? Lightweight AI, Instant Impact 💡

Mu is a transformer-based **encoder–decoder model** with only **330 million parameters**, yet it delivers **100+ tokens per second** and response times under 500 ms, all processed locally on your device (\[blogs.windows.com]\[1], \[windowscentral.com]\[2]). This low latency is crucial for providing instant, fluid responses as users type configuration queries into Settings.

Thanks to techniques such as **dual LayerNorm**, **Rotary Positional Embeddings**, **Grouped‑Query Attention**, and efficient quantization using 8 to 16-bit PTQ, Mu achieves a **47% reduction in first-token latency** and **4.7× faster decoding speed** compared to similar-sized decoder-only models (\[blogs.windows.com]\[1]).

## Deep Optimization for NPUs

Mu was built from the ground up to fully utilize hardware acceleration:

* The **encoder–decoder architecture** separates input and output processing, reducing both computation and memory usage (\[blogs.windows.com]\[1]).
* Model components, such as layer dimensions and operator types, are precisely tuned to match NPU vectorization and memory access patterns (\[blogs.windows.com]\[1]).
* Combined with **post-training quantization (PTQ)** and **optimizations from partners** like AMD, Intel, and Qualcomm, Mu ensures consistently smooth performance across Copilot+ devices (\[blogs.windows.com]\[1]).

## Task‑Focused Fine‑Tuning

Mu is not a general-purpose conversational assistant. It is fine-tuned specifically for Windows Settings:

* Pre-trained on **hundreds of billions of educational tokens**, then distilled from larger Phi models (\[blogs.windows.com]\[1]).
* Fine-tuned using **3.6 million synthetic training samples** across a wide range of settings categories, allowing Mu to achieve accuracy levels comparable to much larger models (\[blogs.windows.com]\[1]).
* It can interpret user requests such as “increase brightness” or “turn off notifications” with context awareness. For vague queries, it smoothly transitions to traditional search results (\[reddit.com]\[3]).

The result is an AI agent within Windows Settings that understands natural language, carries out actual configuration changes, and responds in under 500 ms (\[blogs.windows.com]\[1]).

## Real‑World Use: AI + Settings = Seamless Control

Windows Insiders in the Dev Channel with Copilot+ PCs can already experience Mu in action. For instance, typing “my mouse pointer is too small” prompts the Settings agent to offer an “apply” button to adjust pointer size, avoiding the need to dig through menus (\[windowscentral.com]\[2]).

This integration of natural language understanding, UI interaction, and local processing redefines how users interact with their PCs. It’s intuitive, direct, and private, with no data sent to the cloud.

## What it Means for You

* **Privacy-first AI**, since all processing happens locally on your PC’s NPU, keeping your queries private.
* **Responsive user experience**, with fast responses under 500 ms that make AI feel like a native part of Settings.
* **Scalable capabilities**, as Microsoft plans to expand Mu’s use into additional Windows features beyond Settings.

## Looking Ahead

Still in beta for Windows Insiders, Mu marks a shift toward **small, task-focused AI models** that run offline. Microsoft will continue improving Mu based on feedback, with potential expansion into broader system functionality. For now, users can expect smoother AI interactions within Windows Settings—and possibly more in the future.

---

Microsoft’s Mu proves that **AI doesn’t have to be huge to be powerful**. With thoughtful design, hardware-aware architecture, and a focused use case, even compact models can make a significant impact. As AI becomes more intelligent and integrated, Mu offers a clear vision of the future: lightweight, fast, and secure.

Stay tuned for updates as Mu evolves, and try a simple phrase in your Settings app to see it in action.
