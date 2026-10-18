# Hardware-free terminal footage. Run from repository root after mkdir -p recordings.
# Requires installed project dependencies, tmux and VHS. Never replace --demo with
# a physical recipe in an unattended recording script.
Output recordings/deployment-cli.mp4
Output recordings/deployment-cli.gif
Require uv
Require tmux
Set Shell bash
Set Width 1280
Set Height 720
Set FontSize 22
Set TypingSpeed 25ms
Type "uv run manimux doctor --demo --gui-port 18088 --deployment-dir .local/video-demo"
Enter
Wait /ManiMux · preflight/
Sleep 2s
Type "uv run manimux up --demo --gui-port 18088 --deployment-dir .local/video-demo"
Enter
Wait /ManiMux · demo_ready/
Sleep 3s
Type "uv run manimux status --deployment-dir .local/video-demo"
Enter
Wait /Synthetic preview only/
Sleep 3s
Type "uv run manimux down --deployment-dir .local/video-demo"
Enter
Wait /ManiMux · stopped/
Sleep 2s
