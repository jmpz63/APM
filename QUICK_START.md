> Deprecated: This Quick Start has been consolidated into README.md.

# APM Quick Start (Deprecated)

This file is intentionally minimal. The single source of truth for onboarding and operations is now the root README.

- Go to: ./README.md
- See sections: "Quick Start (Read this first)" and "Current project status"

For AI assistants
```python
from _ADMIN.ai_agent_interface import notify_new_document, ensure_compliance
notify_new_document("path/to/file.md", "description", "priority")
ensure_compliance()
```

CLI helpers
```bash
python3 _ADMIN/ai_agent_interface.py status
python3 _ADMIN/ai_agent_interface.py integrate
```

Last updated: 2025-10-09