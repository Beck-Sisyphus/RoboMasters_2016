# Introduction for Visual Target Detection algorithm

## Function 
Based on cameras installed on the Vehicles, detect enemy vehicle in the visual range.

## Algorithm
### Target Detection and Distinction
1. Color Detection
Collect massive graphic data for enemy automaton, and make training. Obtain the color range of enemy automaton and make color detection on the whole graph by setting this as threshold. Find regions that probably contain enemy automaton and quantize into binary image.
2. Filter out Image Noise
Make “Opening” operation on obtained binary image, and filter out the image noise in the image detection solution.
3. Connected Component Detection
Make a connected component detection on the suspected regions that might have enemy. Find out each region’s out contour, and represent it by a similar rectangle. 
4. Connected Component combination
Based on the distance and color similarity between Connected Components, combine those Connected Components that is close to each other and have high color similarity.
5. Shape and Size Filter
Train on large number of enemy automaton graphs. Get enemy automaton’s shape information (such as length width ratio) and size information (area), and filter the unqualified region based on these data.
After five operations above, we can preliminary get enemy automation’s position, size and shape information.
### Target Motion Tracking 
Make motion tracking based on the detection solution form previous step.
Status Estimate
Based on automaton’s motion status in the last moment (includes position and speed), estimate automaton’s current motion status.
Tracks Correlations
Based on position and color information, correlate current automaton’s estimate status and the detection result.
Status update
If the correlation is success in the last step, then update the current motion status.
By making motion tracking on detection result, we can calculate enemy automation’s moving speed and moving direction.
### Pre-estimate advance quantity
1. Estimate Time Latency
Based on the actual debugging situation of our automaton, use the method that make large number of experience and collect the data to estimate the time latency between the time automaton receiving command to the time the shell (or bullet) is hitting the target (Include graphic operation time, Fall time and shell projectile time).
2. Calculate advance quantity
Based on time latency and enemy automaton’s moving speed, calculate the advance quantity of shell launch. Compensate into enemy automaton’s moving status.

## Summary
There are so many ways to realize enemy detection in automaton. Visual detection is one of them, and target detection based on color is one of simpler and effective way in visual algorithm. So, this paragraph of code is just a inspiring sample which is only used on battlefields and vehicles 2014 RoboMasters summer Camp. We hope to see more simple and effective algorithm from you.
