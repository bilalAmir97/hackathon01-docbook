---
id: 1
title: Create Textbook Constitution
stage: constitution
date_iso: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "setup", "textbook", "physical-ai", "humanoid-robotics"]
spec: null
ticket: null
adr: null
pr: null
outcome: "Successfully created project constitution for Physical AI & Humanoid Robotics textbook"
evaluation: "Constitution file properly created with all required principles, standards, and constraints"
---

# Prompt

```
/sp.constitution

Project: Textbook for Teaching Physical AI & Humanoid Robotics

Description: A comprehensive, AI-native textbook written using Docusaurus and deployed to GitHub Pages. The content bridges the gap between digital intelligence and physical embodiment, guiding students through ROS 2, Gazebo, and NVIDIA Isaac to build autonomous humanoid robots.

Core Principles:
- Embodied Intelligence: All concepts must link software logic (brain) to physical action (body).
- Simulation-First: Prioritize "Sim-to-Real" workflows using Gazebo and Isaac Sim before hardware implementation.
- Technical Rigor: Ensure all code (Python/rclpy, URDF, C++) is syntactically correct and reproducible.
- Clarity & Structure: Content must be modular, student-friendly, and optimized for digital reading (Docusaurus).

Key Standards:
- Platform: Docusaurus (Markdown/MDX) deployed to GitHub Pages.
- Content Structure: Each chapter must use H1 (Title), H2 (Major Sections), and H3 (Subsections) hierarchy [memory:4]. Every chapter must include a concise "Meta Description" (< 160 chars) for SEO [memory:4]. Code blocks must include language tags (e.g., ```)
- Citation & Verification: APA style for theoretical concepts. Direct links to official documentation (ROS 2 Humble, NVIDIA Isaac, Gazebo) for technical claims.
- Accessibility: Flesch-Kincaid Grade 10-12 readability.

Constraints:
- Scope: 4 Core Modules (The Robotic Nervous System, The Digital Twin, The AI-Robot Brain, Vision-Language-Action).
- Word Count: Approximately 1,200-1,500 words per module (Total ~5,000-7,000 words).
- Context Limit: Each individual chapter/section should be concise (~500 words) to maintain focus.[1]
- Tech Stack: ROS 2, Gazebo, NVIDIA Isaac Sim, OpenAI Whisper.

Success Criteria:
- Complete coverage of all 4 modules with verified "Voice-to-Action" code examples.
- Zero broken links or hallucinations in technical commands.
- Passes automated build and linting checks.
```

# Response

## Constitution Created Successfully

I've created the project constitution file at `.specify/memory/constitution.md` with the following structure:

### Core Principles
1. Embodied Intelligence - linking software logic to physical action
2. Simulation-First Approach - prioritizing Sim-to-Real workflows
3. Technical Rigor - ensuring code correctness and reproducibility
4. Clarity & Structure - modular, student-friendly content
5. Accessibility & Readability - maintaining Grade 10-12 level
6. Verification & Citation - using APA style and official documentation

### Standards Defined
- Docusaurus platform with proper content structure
- Module structure with length requirements
- Technology stack requirements (ROS 2, Gazebo, Isaac Sim, OpenAI Whisper)
- Quality assurance standards

### Files Created
- `.specify/memory/constitution.md` - Complete project constitution

The constitution aligns with all specified requirements including the 4 core modules (The Robotic Nervous System, The Digital Twin, The AI-Robot Brain, Vision-Language-Action), technical stack requirements, and content standards.

## Follow-up Actions
- Ready to proceed with textbook development following these principles
- Next steps might include creating feature specifications for each module