January 16, 2011
Robert Hodgin (http://roberthodgin.com)
Hello and welcome to my first Kinect toy: Body Dysmorphia.
Many thanks to Adafruit, OpenKinect, Cinder, and Andrew Bell.


• This application is currently only available for Mac.
• Plug your Kinect into a power outlet, then plug the USB cable into your computer. Wait a few seconds before launching the app. Once the green light on the front of the Kinect is visible (either blinking or solid), launch the app.
• Body Dysmorphia was created on a MacBook Pro 2.66 GHz Intel Core i7 with 8GB of memory. This is the only computer it has been tested on, but should work fine for any reasonably recent Mac.
• Body Dysmorphia was developed using Cinder C++ framework (http://libcinder.org).
• This app is presented as-is. I have not done any extensive testing.


TIPS FOR A BETTER EXPERIENCE
• Try to stand between 5' and 10' from the Kinect. I find this to be an optimal distance.
• Cover or remove reflective objects that might be behind you. Reflective or shiny objects will distort the depth data the Kinect receives.
• If you are in low light conditions, you can either adjust the app's brightness using the 'c' key or switch to infrared mode with the 'i' key.
• Because the Kinect's RGB camera and depth camera are offset from each other, their corresponding images do not align exactly. I have not figured out a great solution for it yet so I have added a parameter called 'Texture X Offset'. It is probably not worth messing with this variable, but if you find the RGB camera image doesn't quite match up with the 3D puffiness geometry, you can try adjusting this 'Texture X Offset' variable using the 'x' key.
• To toggle in and out of presentation mode, just hit the ESC key. 


KINECT QUIRKS TO BE AWARE OF
• The Kinect doesn't like pointing at shiny surfaces. When the Kinect tries to read the distance to a shiny surface, it gets confused and sends back horribly inaccurate data. This usually isn't a problem but it can be the cause of some less than desirable visual oddities. If you see weird 3D data messing up the visuals, try situating the Kinect so that it isn't pointing at any windows, monitors, metal, etc.
• The Kinect has a useful range between about 2' and 20' (don't quote me on that, these are merely estimates based on how I use the Kinect). If any part of you is closer than 2', it will disappear. If you move beyond 20' or so, the depth data starts to blend together and the visual effect will be lessened.



