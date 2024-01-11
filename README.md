# Pickle Coding Challenge

Gherkin is a two-link robot arm trying to reach a randomized goal. Help Gherkin by adding features and improving the existing sandbox.

Your task is to familiarize yourself with the code, review the backlog of tasks, choose one or more projects (or pick one of your own invention), and complete them within a limited time period.

**Note:** There are both Python and JavaScript implementations. They are equivalent. Please choose the language you are most comfortable with and most appropriate for the role you are applying for!

## Goal of Challenge

Our goal is not only to evaluate your coding abilities, but also to get a feel for how you pick up unfamiliar projects, prioritize work, and communicate your decisions with peers. The code in this repo is intentionally imperfect, but functional. The purpose is to provide an example of a real world product, warts and all.

This is your chance to impress us, so choose something(s) that will demonstrate your full programming abilities. We will be looking at both the functionality of your submission and the quality and readabilty of your code. Part of the challenge is to demostrate your ability to tactically choose scope and requirements. It is better to do one thing well than multiple things poorly.

For each project, the Gherkin should continue to reach the goal after implementation.

Note: It is possible you will not be able to complete your chosen task in the time we ask of you. This is fine. Since this is an open ended project, we are evaluating you on your ability to organize and communicate as much as on your coding abilities. Do your best, and be prepared to discuss what worked and what didn't. If you get stuck and want to ask for guidance or help it won't count against you.

# Backlog

**Note:** Please choose items from this list you are comfortable with and show off your fit for the role you are applying to.

If you have a great idea that isn't on this list, feel free to do that instead.

* Add walls and obstacles to the world
  * Bonus: make a more easily configurable representation
* Add a third link to the arm
* Make a moving goal
* Improve/Overhaul the graphics or simulation systems (3D, flashier, whatever you want)
* Visualize Paths, Velocities, Accelerations, over time in the sandbox window
  * Note: Not as good to render several separate visualizations
* "Cloudify" gherkin's configuration
* Use a Behavior Tree to make complex behaviors
* Add unit tests and clean up the code (note: don't just do this item!)
* Change system from top down a side view - add gravity!
  * This will require changing Gherkin to use torque control
* Fix sloppy coding
  * We've intentionally left some cruft behind. Bonus points for showing us our mistakes.

## NodeJS Specific 
  * Port to TypeScript
  * Improve React usage or change to a different framework (threejs, etc)

# Submission

**Please submit your changes as a pull request to the repository.**

The implementation can leverage any library you choose or code you write. Just make sure it works when we check out the repository and install dependencies with the respective build systems.

We'll be taking a look at the quality of your code and how you use version control. Please make your PR easy to review! We will also read your commit messages, briefly examine the history of the repository, etc.

This exercise is "open-book", so take the time you need to craft a careful and correct solution. Please feel free to use whatever tools, techniques, or materials you care to. Incorporating code from elsewhere is fine, but please credit the author(s).

If you have any questions about the exercise or would like further clarification on the requirements feel free to reach out!

# Python

Code is located in the `python` directory.

We have tested this code on recent (3.9+) versions of Python, but please let us know if you run into issues.

### Setup the Development Environment

```
$ ./scripts/bootstrap
```

### Enter the virtual environment

```
$ source ./scripts/init
```

### Run

```
$ python src/challenge.py
```

# JavaScript

Code is located in the `javascript` directory.

Any reasonable version of NodeJS should work. We have tested this with NodeJS 18.16.0.

## NodeJS Development Crash Course
If you are new to NodeJS development, here's a crash course that should get you started:

### Install nodejs for your platform

There are various ways to do this:

* Mac with Homebrew - brew install node
* Everything else - https://nodejs.org/en/download/

## Setup and Run

Note: While we expect the environment and sandbox to work across different platforms, we have not done extensive testing.

We have tested this with NodeJS 18.16.0

### Setup the Development Environment

```
$ npm ci
```

### Run

```
$ npm start
```
