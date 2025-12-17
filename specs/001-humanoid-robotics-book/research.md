# Research for "Physical AI & Humanoid Robotics" Book

This document outlines the research tasks required to finalize the technical details of the book.

## 1. Technology Versions

### Decision:
The book will standardize on the following Long-Term Support (LTS) and stable versions of the core technologies:
- **ROS 2**: Humble Hawksbill (LTS)
- **Gazebo**: Fortress
- **Unity**: 2022.3 LTS
- **NVIDIA Isaac Sim**: 2023.1.1
- **Python**: 3.10

### Rationale:
Using LTS and recent stable versions provides a balance of modern features and stability, which is crucial for a book that will be used over several years. This minimizes the risk of bit-rot and ensures that readers have a consistent experience.

### Alternatives Considered:
- **Using the latest rolling releases**: This would provide the most modern features, but at the cost of stability and a higher likelihood of breaking changes.
- **Using older, more established versions**: This would maximize stability but would mean that the book is not teaching the most current practices.

## 2. LLM Integration with ROS 2

### Research Task:
Investigate and document best practices for integrating Large Language Models (LLMs) with ROS 2 for cognitive planning.

### Key Areas of Investigation:
- **Communication patterns**: Should the LLM be called as a service, or should it be a standalone node that subscribes to topics?
- **Prompt engineering**: How to structure prompts to elicit useful, structured data from the LLM that can be translated into robotic actions.
- **Action translation**: How to translate the natural language output of an LLM into a sequence of ROS 2 actions.
- **Error handling**: How to handle cases where the LLM produces invalid or unsafe commands.

### Outcome:
A dedicated chapter in the book that provides a clear, step-by-step guide to integrating an LLM for cognitive planning, including code examples and best practices.
