# info from https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html#override-board-configuration
Import("env")

board_config = env.BoardConfig()
# should be array of VID,PID pairs
board_config.update("build.hwids", [
  ["1d50", "6017"], #BMP
])
'''
board_config.update("build.hwids", [
  ["1d50", "6017"], #BMP
  ["1d50", "6017"] #DFU VID:PID, enkel nodig als <> van eerste pair
])
'''
