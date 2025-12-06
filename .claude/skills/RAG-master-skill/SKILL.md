# Skill: RAG Master Skill

## Purpose
This skill equips the RAG Agent for the book *Physical AI & Humanoid Robotics* to perform full Retrieval-Augmented Generation (RAG) workflows. It covers:

- Preprocessing and semantic chunking of text  
- Embeddings generation for vector databases  
- Vector database querying (Qdrant)  
- Grounded reasoning and structured answer generation  
- User-selected text handling  
- Multilingual support (Urdu)  
- Citation and reasoning trace generation  

This is a **comprehensive, modular, production-ready RAG skill** suitable for Claude Code.

---

# Capabilities

1. **Preprocessing & Chunking**
   - Normalize book content, remove artifacts, maintain headings, tables, formulas.
   - Chunk text into semantic units (300–900 chars) while preserving context.
   - Attach metadata: chapter, section, heading, source.

2. **Embedding Generation**
   - Generate vector embeddings per chunk (recommend `text-embedding-3-large`).
   - Preserve technical terms, formulas, and multilingual content.
   - Maintain metadata for retrieval filtering and traceability.

3. **Vector Database Querying**
   - Embed user questions.
   - Query Qdrant with top_k selection and metadata filters.
   - Rank retrieved chunks by semantic relevance.
   - Return chunk IDs, text, scores, and metadata.

4. **Context Grounding**
   - Assemble retrieved or selected chunks into coherent, factual answers.
   - Always provide citations referencing chunk IDs or headings.
   - Include structured JSON output: answer, sources, reasoning trace.
   - Prevent hallucination; use only retrieved or selected content.

5. **User-Selected Text Handling**
   - Detect user-highlighted text and treat it as primary source.
   - Override vector DB retrieval if selection exists.
   - Maintain all technical content and formatting.
   - Generate answers grounded solely in the selected text.

6. **Multilingual Support**
   - Translate final answers to Urdu if requested.
   - Preserve technical terms in English or transliteration.
   - Maintain JSON output consistency.

7. **Structured Output**
   - Always return JSON-safe output:
```json
{
  "answer": "...",
  "sources": [
    {"id": "chunk_12", "chapter": "6"}
  ],
  "reasoning_trace": [
    "retrieved 3 chunks",
    "used top 2 for answer"
  ]
}
