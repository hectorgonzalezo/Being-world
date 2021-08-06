import OSC

c = OSC.OSCClient()
c.connect(('127.0.0.1', 57120))   # localhost, port 57120
oscmsg = OSC.OSCMessage()
oscmsg.setAddress("/startup")
oscmsg.append('HELLO')
c.send(oscmsg)
