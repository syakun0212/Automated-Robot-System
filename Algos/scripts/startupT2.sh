lxterminal -e 'bash -c "cd ~/mdp_ws/src/Integrate && python3 hwInterface.py"'
sleep 2s
xterm -e 'cd ~/mdp_ws/src/Integrate && python3 ParticleUpdater.py'
sleep 2s
xterm -e 'cd ~/mdp_ws/src/Integrate && python3 BTservice.py'
sleep 2s
xterm -e 'cd ~/mdp_ws/src/Integrate && python3 commandAutoT2.py'
sleep 2s
xterm -e 'cd ~/mdp_ws/src/Integrate && python3 imageDetectionObst.py'
sleep 2s
xterm -e 'cd ~/mdp_ws/src/controller2 && python3 controller4.py'
