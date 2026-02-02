# MCP Server Configuration Complete

## Overview
Your VS Code workspace is now optimized with 4 MCP servers tailored for the NOMAD drone project.

## Configured Servers

### 1. **Forgetful** - Semantic Memory System
**Status**: ✅ Configured  
**Purpose**: Stores architectural patterns, API conventions, and project-specific knowledge  

**Optimizations**:
- Database stored at `.mcp/forgetful.db` (local, fast access)
- Project name: "NOMAD"
- Project type: "drone-autonomy"
- Will remember:
  - Task 1 vs Task 2 architectural differences
  - API endpoint patterns (`/api/task/1/*`, `/api/task/2/*`)
  - MAVLink communication patterns
  - Jetson ↔ Ground Station architecture

**Usage**: Automatically captures important context as you work

---

### 2. **GitHub MCP Server**
**Status**: ✅ Configured  
**Purpose**: GitHub repository operations, PR management, issue tracking  

**Optimizations**:
- HTTP connection via GitHub Copilot API (reliable)
- Default repository hint: "MAD/NOMAD"
- Version: 0.30.2

**Usage**: Integrates with GitHub operations in VS Code Copilot

---

### 3. **Serena** - Advanced Coding Agent
**Status**: ✅ Configured  
**Purpose**: Semantic code understanding, multi-file refactoring, symbol navigation  

**Optimizations**:
- **Workspace path**: `${workspaceFolder}` (automatically resolved)
- **Indexed directories**: 
  - `edge_core/` (Python FastAPI backend)
  - `mission_planner/src/` (C# plugin)
  - `tailscale/src/` (Network monitoring)
- **Languages**: Python, C#, YAML, Dockerfile
- **Frameworks**: FastAPI, ROS2, .NET
- **Memory path**: `.mcp/serena/` (local cache)

**Key Features**:
- Symbol-level code navigation
- Understands cross-file dependencies
- Optimized for large refactoring operations
- IDE assistant context for code editing

**Usage**: Use for complex code analysis and refactoring tasks

---

### 4. **Context7** - Documentation Library
**Status**: ✅ Configured (requires API key)  
**Purpose**: Provides up-to-date library documentation and API references  

**Optimizations**:
- **Default libraries** configured:
  - `fastapi` - Web framework for edge_core
  - `ros2` - Robotics framework
  - `pymavlink` - MAVLink protocol library
  - `ultralytics` - YOLO object detection
  - `opencv-python` - Computer vision
  - `pydantic` - Data validation models

**Usage**: Provides instant documentation when working with these libraries

**First Time Setup**:
1. You'll be prompted for `CONTEXT7_API_KEY` on first use
2. Get your API key from: https://context7.ai
3. Key is stored securely in VS Code

---

## File Structure Created

```
NOMAD/
├── .vscode/
│   ├── settings.json           # ✅ Workspace settings + MCP integration
│   └── mcp-settings.json       # ✅ Detailed MCP server configs
├── .mcp/
│   ├── README.md               # ✅ Documentation
│   ├── forgetful.db            # (generated on first use)
│   └── serena/                 # (generated on first use)
└── .gitignore                  # ✅ Updated to exclude MCP data
```

---

## VS Code Settings Enhanced

In addition to MCP servers, your [.vscode/settings.json](.vscode/settings.json) now includes:

### Python Configuration
- ✅ Analysis paths: `edge_core/`, `tailscale/src/`
- ✅ Type checking: basic mode
- ✅ Default interpreter path

### C# Configuration
- ✅ OmniSharp EditorConfig support
- ✅ Roslyn analyzers enabled

### File Associations
- ✅ `.param` → properties (ArduPilot params)
- ✅ `.env` → properties (environment configs)
- ✅ `.conf` → properties (MAVLink router)

### Search Exclusions
- ✅ Excludes: `__pycache__`, `bin/`, `obj/`, `tmpclaude-*`, `.vs/`

---

## Next Steps

### 1. Install Dependencies (if not already installed)
```powershell
# Install uvx (for Python-based MCP servers)
pip install uvx

# Install npx (comes with Node.js)
# Download from: https://nodejs.org/
```

### 2. Get Context7 API Key
1. Visit: https://context7.ai
2. Sign up and get your API key
3. First time you use Context7 in VS Code, you'll be prompted

### 3. Restart VS Code
```powershell
# Reload VS Code to activate MCP servers
# Press: Ctrl+Shift+P → "Developer: Reload Window"
```

### 4. Verify MCP Servers
Open GitHub Copilot chat and ask about:
- "What API endpoints does NOMAD have?" (tests Forgetful/Serena)
- "Show FastAPI documentation for async routes" (tests Context7)

---

## Maintenance

### Clear MCP Cache
```powershell
# Remove cached data (safe, will regenerate)
Remove-Item -Recurse .mcp\forgetful.db
Remove-Item -Recurse .mcp\serena\
```

### Update MCP Server Versions
Edit [.vscode/mcp-settings.json](.vscode/mcp-settings.json) and update version numbers.

### Disable a Server Temporarily
Comment out the server in [.vscode/settings.json](.vscode/settings.json):
```json
// "forgetful": { ... }
```

---

## Troubleshooting

### Issue: MCP servers not loading
**Solution**: 
1. Ensure `uvx` and `npx` are installed
2. Check VS Code Output panel → "GitHub Copilot"
3. Reload window: `Ctrl+Shift+P` → "Developer: Reload Window"

### Issue: Context7 prompts for API key repeatedly
**Solution**: 
1. Check the key is correct at https://context7.ai/dashboard
2. Re-enter when prompted
3. Key is stored in VS Code secure storage

### Issue: Serena index out of date
**Solution**: 
1. Delete `.mcp/serena/`
2. Reload VS Code to rebuild index

---

## Project-Specific Tips

### For Edge Core Development (Python)
- **Use Context7** for FastAPI, pymavlink, pydantic docs
- **Use Serena** for navigating API endpoints and VIO pipeline
- **Use Forgetful** to remember MAVLink message patterns

### For Mission Planner Plugin (C#)
- **Use Serena** for understanding C# plugin architecture
- **Use Context7** for .NET documentation
- **Use GitHub MCP** for PR creation and review

### For System Integration
- **Use Forgetful** to track Jetson ↔ GCS communication patterns
- **Use Serena** for cross-language refactoring (Python ↔ C#)

---

## Summary of Changes

| File | Status | Purpose |
|------|--------|---------|
| `.vscode/settings.json` | ✅ Created | Workspace settings + MCP integration |
| `.vscode/mcp-settings.json` | ✅ Created | Detailed MCP server configurations |
| `.mcp/README.md` | ✅ Created | MCP documentation |
| `.gitignore` | ✅ Updated | Excludes MCP data, keeps README |

**All configurations are workspace-scoped and won't affect other projects.**

---

*Configuration complete! MCP servers will activate on next VS Code reload.*  
*NOMAD - McGill Aerial Design AEAC 2026*
