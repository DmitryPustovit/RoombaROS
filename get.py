#!/usr/bin/env python
import math
import sys
import requests
import threading
def getCommand():
  threading.Timer(0.2, getCommand).start()
  r = requests.get("https://turtle-ui.herokuapp.com/command", verify=False).json()
  if(r == "0"):
  	print(r)
getCommand()
