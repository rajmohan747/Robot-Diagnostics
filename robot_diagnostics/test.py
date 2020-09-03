import roslaunch

package = 'robot_diagnostics'
executable = 'dummyBattery'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
while True:
   print process.is_alive()

