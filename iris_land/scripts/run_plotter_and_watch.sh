#!/bin/bash
roslaunch drone_ekf plotter.launch

echo ""
echo "Plotter launch exited."
echo "Watching detached saver log. Press Ctrl+C to stop watching."
echo ""

tail -f /tmp/ekf_plots/plotter_shutdown_save.log
