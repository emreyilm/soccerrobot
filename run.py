import math
import sys

from robotControl import *

class Runner():
	def __init__(self):
		self.main=robotController()
	
	def start_game(self):
		self.main.GameLoop()

if __name__ == '__main__':
	s=Runner()
	game_choice=raw_input("Please press a button to initiate the game:")
	s.start_game()
	s.mainer.IsGameStarted=False
	

	