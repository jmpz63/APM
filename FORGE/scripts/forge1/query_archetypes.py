#!/usr/bin/env python3
"""Query forge1 chroma collections for the 'website archetypes' RAG doc."""
import chromadb

for db_path in ("/home/jmpz63/forge_zero/chroma_db", "/home/jmpz63/forge_chroma_db"):
    try:
        client = chromadb.PersistentClient(path=db_path)
    except Exception as e:
        print(f"! cannot open {db_path}: {e}")
        continue
    for col in client.list_collections():
        try:
            c = client.get_collection(col.name)
            data = c.get(include=["documents", "metadatas"])
            hits = 0
            for doc, meta in zip(data["documents"], data["metadatas"]):
                low = (doc or "").lower()
                if "archetype" in low or "archtype" in low or "architype" in low:
                    hits += 1
                    src = (meta or {}).get("source", "?")
                    print(f"=== {db_path} :: {col.name} :: {src}")
                    print(doc[:1500])
                    print("---")
                    if hits >= 3:
                        break
        except Exception as e:
            print(f"! {col.name}: {e}")
