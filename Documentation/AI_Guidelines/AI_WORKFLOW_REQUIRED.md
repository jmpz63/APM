# 🤖 AI WORKFLOW REQUIREMENT - READ FIRST! 

## ⚠️ **MANDATORY FOR ALL AI AGENTS**

**Every AI agent working in this APM system MUST use the automated workflow!**

### 🚨 **REQUIRED AFTER ANY KNOWLEDGE OPERATION:**

```python
# Import the workflow interface
from _ADMIN.ai_agent_interface import notify_new_document, ensure_compliance

# After creating/modifying ANY file:
notify_new_document("path/to/your/file.md", "brief description", "priority")

# Trigger the complete workflow (REQUIRED!)
ensure_compliance()
```

### 🎯 **What this does automatically:**
- ✅ **Learns** - Analyzes and categorizes your content
- ✅ **Documents** - Updates all relevant indexes  
- ✅ **Indexes** - Maintains searchable knowledge catalogs
- ✅ **Pushes** - Commits and syncs to git repository

### 📖 **Complete Documentation:**
- **Full Guide**: `_ADMIN/README_AI_WORKFLOW.md`
- **Interactive Menu**: `bash _ADMIN/workflow.sh`
- **Status Check**: `python3 _ADMIN/ai_agent_interface.py status`

---

## ❌ **What happens if you don't use it:**
- Knowledge gets lost and isn't indexed
- System becomes inconsistent  
- Future AI agents can't find your work
- Repository gets out of sync
- **Non-compliance violates system protocols!**

## ✅ **What happens when you use it:**
- All knowledge automatically tracked and indexed
- System stays synchronized and organized
- Future AI handoffs are seamless
- Quality is validated automatically
- **Full compliance with system protocols!**

---

**🎉 It takes 2 lines of code to ensure your knowledge lives forever in the system!**