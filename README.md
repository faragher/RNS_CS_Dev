# RNS C# Port - Pre-alpha testing repository

## Please actually read this.

A C# port of the Reticulum Network Stack - https://github.com/markqvist/Reticulum

This port is in the very early stages. In fact, it does basically *nothing* that Reticulum does. There has been some interest in seeing the C# interface driver for the RNode, which is largely functional for testing. Almost all other parts of Reticulum are abstracted or simply dummied out.

### What can it do?
* It can initialize the RNode, turn the backlight on and off, work the frame buffer, and send and receive arbitrary data.
* Use an event handler to call an arbitrary number of callbacks.

### What can't it do?
Route, encrypt, split packets, process announcements, really anything but act as a little RNode interface system. For running the RNode as a little packet radio, it's great. For everything else, it's utterly incapable.

### Known bugs:
* Occasionally the system reports the bandwidth as 65535, an issue I haven't tracked down, but likely an issue with serial timing. The radio will try to reinitialize if you simply wait.
* You may have to forcibly terminate the program or cycle the RNode in certain failure modes. I have not done extensive testing prior to release.

### Current use case:
Program.cs contains the core loop which is compatible with the RNode example Python script. https://github.com/markqvist/RNode_Firmware/tree/master/Python%20Module

### Disclaimer:
I am attemping to port the RNS, and it's a long project. I will attempt to track down bugs if they're reported, but please be advised I am not likely to prioritize QOL or features over time spent on the stack itself. Tested on Windows, but intended to be Windows/Linux compatible and possibly supported on Android in the future.

Strange decisions in variables are a combination of attempting to use appropriate variable sizes instead of making everything an INT32 or similar, and simply trying to work around the abstraction Python uses for variables. This is in preparation for porting to microcontrollers using C/C++, likely to be performed by another programmer.

Certain features, such as the backlight, may be implemented here and not in the stock RNS. With the exception of capitalization and other C# specific changes, full API compatibility is intended, but temporary differences between the versions is unavoidable and internal mechanisms may differ.

Progress will be sporadic. Thanks for supporting Reticulum.
