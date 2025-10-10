# AI Operations Guide (APM)

Purpose: Give new AI agents a fast, reliable playbook to operate within APM without breaking conventions.

## Golden rules
- Always use the workflow: `_ADMIN/README_AI_WORKFLOW.md` (notify_new_document → ensure_compliance)
- APM is the knowledge base and project hub; active work lives in working folders/repos
- Cross-link both ways: Project hub ↔ working folder/repo
- Minimize churn: small, focused commits with clear messages
- Verify paths and repos before editing; never assume

## First 10 minutes checklist
1) Read `_ADMIN/README_AI_WORKFLOW.md` and this guide
2) Skim `00_MASTER_KNOWLEDGE_INDEX.md` and `_ADMIN/todo.md`
3) Check `PROJECTS/Active/*` hubs exist and include Roles & Cross-links
4) Confirm working folders exist (Windows) or live paths (ARM1) and link them
5) If creating a new hub, use the template in `Tools/Templates/PROJECT_HUB_README_TEMPLATE.md`

## Project hubs (Active projects)
- Include a "Roles & Cross-links" section:
  - APM hub: KB/planning; curated outputs only
  - Working folder (Windows path) and/or live path (e.g., ARM1)
  - Remote repo link if any
- Keep heavy assets out of APM; link to source of truth

## Submodules and mirrors
- If a project has its own repo, add it as a submodule under the project hub `/repo`
- Document the local working clone path and the GitHub URL in the hub README

## Environment awareness
- Windows local paths: `C:\\Users\\jmpz6\\OneDrive\\Moveo\\...`
- ARM1 live paths: `/home/arm1/...` (e.g., Klipper at `/home/arm1/printer_data/config/printer.cfg`)
- Don’t push device-local configs into APM unless they’re docs or snapshots

## Safety & quality gates
- Validate edits compile or lint where applicable; keep example code runnable
- Prefer minimal, non-breaking changes; avoid mass reformat
- Use `DUMP_TMC` and documented procedures for hardware-related steps (see TMC KB)

## Commit etiquette
- Group related change sets; avoid mixed-purpose commits
- Prefix messages with area: `docs(project-hubs): ...`, `knowledge(TMC): ...`, `projects(...): ...`
- Push after ensure_compliance() processes the change

## Quick references
- Workflow: `_ADMIN/README_AI_WORKFLOW.md`
- Master index: `00_MASTER_KNOWLEDGE_INDEX.md`
- TMC KB: `Knowledge_Base/Engineering_Expertise/Motion_Control/TMC/`
- WallPanelization hub: `PROJECTS/Active/WallPanelization/`

---
Last updated: 2025-10-09
