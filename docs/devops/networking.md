# Networking Tools for Robotics

## Tmux and Tmuxp

### Tmux

Tmux is a commonly used terminal multiplexer. This program allows you to split one terminal into many.
When you run `tmux`, you create a tmux session. This session can survive interrupted network connections,
so if you lose your internet, you can pick up right where you left off by reattaching to your still-running session.

In each session, you can have multiple windows (selectable like tabs in a web browser), and in each window,
you can have multiple panes. This lets you have several terminals side-by-side in just one! Isn't that handy?

### Tmuxp

Tmuxp is a session manager for tmux. It's built on top of a Python library for tmux, called libtmux. 
Tmuxp makes it easy to launch preconfigured tmux sessions with the panes and windows you want already loaded.
You can even set it to run commands! With built-in delays too!

## SSH

SSH, or Secure Shell, allows you to connect to computers remotely through your terminal.
When you combine tmux with SSH, you end up with a robust way for controlling a remote computer.

## ZeroTier

ZeroTier is a service that allows for creating virtual LAN networks between computers through a technique called 
UDP holepunching. This allows for computers on different networks that are connected to the internet to behave
as if they are on the same networking, saving a lot of headache on port-forwarding and other issues when remoting
into devices.

## Husarnet

Husarnet is similar to ZeroTier, but developed specifically with ROS in mind. This is yet to be tested, but it
should have the ability to connect devices on the same network even without an internet connection.
