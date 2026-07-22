# Website Build Process — Cleaning Queens KC (traceable pipeline)

How the site was actually built on 2026-07-22: every knowledge source,
query, decision point, and the artifacts you can pick up ("pickoff points")
to trace or resume any stage. Companion to `Website_Bible.md`.

Legend: 🟪 pickoff point (artifact on disk/git you can resume from) ·
🔶 decision gate · 🟩 external knowledge in.

```mermaid
flowchart TB

subgraph P0["📥 Phase 0 — Intake & Coordination"]
  A0["Client request:<br/>pink/black site + admin suite<br/>for The Cleaning Queens"] --> A1["APM agent claim<br/><i>Tools/agent_status.py claim</i>"]
end

subgraph P1["🧠 Phase 1 — Knowledge Gathering"]
  B0["RAG query: forge1 knowledge base<br/><i>'website archetypes / build standards'</i>"]
  B0 --> B1{"🔶 Doc exists<br/>in RAG?"}
  B1 -- "MISS" --> B2["Write the reference material:<br/>WEBSITE BIBLE<br/>archetypes · conversion spine ·<br/>design laws · legal boundaries"]
  B1 -- "HIT (future builds)" --> B9["Reuse Bible from RAG"]
  B2 --> B3[/"🟪 Business/Intelligence/<br/>Website_Bible.md"/]

  C0["Web fetch: client's Facebook page"] --> C1{"🔶 Blocked?"}
  C1 -- "yes (login wall)" --> C2["Headless Playwright scrape<br/>from forge1 (Chrome UA)"]
  C1 -- "no" --> C3
  C2 --> C3["Extracted: phones · email · tagline ·<br/>about text · services · 100% recommend ·<br/>real review comment"]
  C2 --> C4["Photo URLs are signed fbcdn links<br/>⚠️ THEY EXPIRE — download immediately"]
  C4 --> C5[/"🟪 previews/cleaning-queens/img/<br/>fb_01 brand banner · fb_03 work photo"/]
  C2 --> C6[/"🟪 scripts/forge1/<br/>scrape_cq_facebook.py<br/>scrape_cq_photos.py"/]
end

subgraph P2["🏗️ Phase 2 — Build (Bible §2–§9 applied)"]
  D0["data.js — CQStore<br/>localStorage store · seeded demo data ·<br/>real business block from scrape"]
  D0 --> D1["index.html — public site<br/>reviews marquee · hero · services ·<br/>booking form · Pearl chatbot"]
  D0 --> D2["admin.html — Queens HQ<br/>dashboard · schedule · jobs kanban ·<br/>clients · payments · reviews · staff"]
  D1 --> D3[/"🟪 FORGE/studio/previews/<br/>cleaning-queens/ (source of truth)"/]
  D2 --> D3
end

subgraph P3["🚀 Phase 3 — Deploy"]
  E0["scp → forge1<br/>/home/jmpz63/forge_studio/previews/"] --> E1["Live: 192.168.50.201:8090<br/>/preview/cleaning-queens/"]
end

subgraph P4["✅ Phase 4 — Verify (gate before ship)"]
  F0["Headless Playwright suite on forge1<br/><i>verify_cleaning_queens.py</i>"] --> F1{"🔶 29/29<br/>pass?"}
  F1 -- "no" --> F2["Fix root cause → redeploy"] --> F0
  F1 -- "yes" --> F3[/"🟪 scripts/forge1/<br/>verify_cleaning_queens.py<br/>(regression suite)"/]
end

subgraph P5["💾 Phase 5 — Persist (nothing lives only in chat)"]
  G0["git commit → clean temp worktree →<br/>push origin master"] --> G1[/"🟪 github.com/jmpz63/APM<br/>2ebf54b → fe06c9b"/]
  G0 --> G2["forge_apm_sync.sh →<br/>RAG auto-reindex"] --> G3[/"🟪 forge1 ~/APM +<br/>searchable knowledge base"/]
  G0 --> G4[/"🟪 Cascade memory<br/>(architecture · creds · gotchas)"/]
  G4 --> G5["APM claim released"]
end

subgraph IT["🔁 Iteration loop (user feedback → trace → fix)"]
  H0["User feedback<br/>(e.g. chatbot trapped in booking loop)"] --> H1["Trace from pickoff point:<br/>screenshot → index.html chat script"]
  H1 --> H2["Fix + add regression test<br/>replaying the exact failure"]
  H2 --> F0
end

A1 --> B0
A1 --> C0
B3 --> D0
C3 --> D0
C5 --> D1
D3 --> E0
E1 --> F0
F1 -- "yes" --> G0
E1 -.-> H0

style B3 fill:#e9d5ff,stroke:#7e22ce,color:#111
style C5 fill:#e9d5ff,stroke:#7e22ce,color:#111
style C6 fill:#e9d5ff,stroke:#7e22ce,color:#111
style D3 fill:#e9d5ff,stroke:#7e22ce,color:#111
style F3 fill:#e9d5ff,stroke:#7e22ce,color:#111
style G1 fill:#e9d5ff,stroke:#7e22ce,color:#111
style G3 fill:#e9d5ff,stroke:#7e22ce,color:#111
style G4 fill:#e9d5ff,stroke:#7e22ce,color:#111
style B1 fill:#fef3c7,stroke:#d97706,color:#111
style C1 fill:#fef3c7,stroke:#d97706,color:#111
style F1 fill:#fef3c7,stroke:#d97706,color:#111
style B0 fill:#dcfce7,stroke:#16a34a,color:#111
style C0 fill:#dcfce7,stroke:#16a34a,color:#111
style C2 fill:#dcfce7,stroke:#16a34a,color:#111
```

## Pickoff points (resume/trace from any of these)

| # | Artifact | What you can do from it |
|---|----------|------------------------|
| 1 | `Business/Intelligence/Website_Bible.md` | Start ANY new client site — archetype, spine, legal rules |
| 2 | `FORGE/scripts/forge1/scrape_cq_facebook.py` / `scrape_cq_photos.py` | Re-scrape client socials for fresh info/photos |
| 3 | `FORGE/studio/previews/cleaning-queens/img/` | Client's real brand assets (fbcdn originals expired) |
| 4 | `FORGE/studio/previews/cleaning-queens/` | Source of truth — edit, then scp to forge1 |
| 5 | `FORGE/scripts/forge1/verify_cleaning_queens.py` | Regression-test any change (29 checks) |
| 6 | `github.com/jmpz63/APM` commits `2ebf54b…fe06c9b` | Full history; diff any iteration |
| 7 | forge1 `~/APM` + RAG index | Agents can query the Bible + this doc |
| 8 | Cascade memory | Session-independent context (creds, gotchas, deploy path) |
