# Running the Code - Simple Solution

## EASIEST METHOD: Use Terminal (Recommended)

Just activate the venv and run your code:

```bash
cd ~/Documents/College/Synthesis/SpatiaLLM/LM2PCG
source venv/bin/activate
python mutli_room_agent2.py
```

You should see `(venv)` appear in your terminal prompt, which means the environment is activated.

### To Run Any of Your Files:

```bash
# After activating venv (source venv/bin/activate):
python mutli_room_agent2.py      # Main AI agent
python room_database.py           # Generate database
python enrich_room_types.py       # Vision classifier
```

### To Deactivate When Done:

```bash
deactivate
```

---

## Alternative: VS Code Interpreter Setup (More Complex)

If you really want to use the Run button in VS Code:

1. Press `Cmd + Shift + P`
2. Type: `Python: Select Interpreter`
3. Click **"Enter interpreter path..."**
4. Click **"Find..."**
5. Navigate to: `/Users/neelabhsingh/Documents/College/Synthesis/SpatiaLLM/LM2PCG/venv/bin/python`
6. Select it

But using the terminal is simpler and always works!
