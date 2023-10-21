# CS-350-Smart-Thermostat

## Summarize the project and what problem it was solving.
This project was to use the CC3200SF launchpad to design and implement the logic for a smart thermostat. This solves the real world problem of collacting and responding to the data receving from the temperature sensor and with conitnued development would be able to connect to a cloud server for remote control over a heating system.

## What did you do particularly well?
I think I did a lot of this project very well, mainly though I think I set up my tasks well and using indivual functions for each task allowed me to create modular and easy to use/understand code.

## Where could you improve?
In terms of raw code an improvement I could make is moving the task scheudler loop out of main and have it just be a function call, either with a global array of tasks or by passing the array to the function. My code also does not allow tasked with a period longer than 1000ms to be used so I could improve on my logic to allow that to happen.

## What tools and/or resources are you adding to your support network? 
I am adding many resources for embedded development as well as resources related to creating a state-machine because that is very relevant in other fields of programming.

## What skills from this project will be particularly transferable to other projects and/or course work?
The capture and convert process is probably the most transferable skill from this project to other projects and I have already started using it in developing programs.

## How did you make this project maintainable, readable, and adaptable?
I did this through a variety of factors, mainly clear and descriptive variable names so the reader is able to understand the process, as well through detailed comments and documenation for each function. To make the project adaptable I create a struct so you can add as many tasks as you want so long as you add the corresponding function for it to call and I used a modudlar/functional style of development so functions can easily be added or reused which helps with maintainablity and adaptability.
