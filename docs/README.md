# A* Optimizations and Improvements
###### Research Work by Lucho Suaya – Universitat Politècnica de Catalunya

## Header
I am Lucho Suaya, a student of the Bahcelor’s Degree in Video Games by [UPC](https://www.upc.edu/ca) at [CITM](https://www.citm.upc.edu/ing/estudis/graus-videojocs/). This content is generated for the second year’s subject Project II, under the supervisión of lecturers [Ricard Pillosu](https://es.linkedin.com/in/ricardpillosu) and [Marc Garrigó](https://www.linkedin.com/in/mgarrigo/).

## Index

  1. [Research Organization](#research-organization)
  2. [Introduction to Problem](#introduction-to-problem)
     * [Before Starting](#before-starting)
  3. [A* First Improvements, Generalities and Context](#a-first-improvements-generalities-and-context)
     * [General Improvements and Heuristics Changes](#general-improvements-and-heuristics-changes)
       * [Beam Search](#beam-search)
       * [Bidirectional Search](#bidirectional-search)
       * [Dynamic Weighting](#dynamic-weighting)
       * [Iterative Deepening (IDA*)](#iterative-deepening-ida)
       * [Building This Section](#building-this-section)
     * [Paths Recalculations - Incremental Searches](#paths-recalculations---incremental-searches)
       * [Fringe Saving A*](#fringe-saving-a)
       * [Generalized Adaptive A* (GAA*) - Initial Approach to Moving Targets](#generalized-adaptive-a-gaa---initial-approach-to-moving-targets)
       * [Dynamic A* (D*) and Lifelong Planning A* (LPA*)](#dynamic-a-d-and-lifelong-planning-a-lpa)
         * [LPA* and D*’s son: A love story](#lpa-and-d-s-son-a-love-story)
      * [Angled Pathfinding](#angled-pathfinding)
        * [Field D*](#field-d)
        * [Theta*](#theta)
        * [Incremental Phi*](#incremental-phi)
  4. [Nowadays - Hierarchies and other Games](#nowadays---hierarchies-and-other-games)
  5. [My Approach - Killing Path Symmetries](#my-approach---killing-path-symmetries)
  6. [Final Thoughts and Recommendations](#final-thoughts-and-recommendations)
  7. [Links to Additional Information](#links-to-additional-information)

## Research Organization
As you may see in the index, this page has many sections and many information. It’s possible that you are not interested in everything that is explained here, so to ease your accessibility, let me explain how the research is organized (apart of introduction part).

   1. First of all, in [A* First Improvements, Generalities and Context](#a-first-improvements-generalities-and-context), I explain the main and minor optimizations for A* and I introduce the concept of Incremental Pathfinding as well as explaining a bit the context in which pathfinding evolved. Finally, I go a bit over angled pathfinding.
  
   2. In the second part, [Nowadays - Hierarchies and other Games](#nowadays---hierarchies-and-other-games) I start with the Hierarchical Pathfinding approach to improve A*, which is very used and common nowadays, and for that reason, I put many examples of other games using it (in fact, it’s difficult to find big games that do not use it).
  
   3. In the 3rd and final part, I introduce and develop [My Approach - Killing Path Symmetries](#my-approach---killing-path-symmetries), to improve A*, which is a way to do it developed by an investigator called Daniel Harabor and makes A* way faster. Also, other improvements are explained since JPS might not be the best take for all games, so to have references if it’s your case (although they are not developed as JPS, I put links to works that show how to do it).
  
Then I conclude this research with [final thoughts](#final-thoughts-and-recommendations) and some [additional information](#links-to-additional-information).

## Introduction to Problem
As you may deduce, in many games, there is a need to find paths from a location to another, for example, to give a path for a unit to move. To do this, we build algorithms in order to find these paths in an automatic manner. In this research, we will talk about one specific algorithm, which I will suppose you already know, the A*, in order to find optimizations and improvements for it.

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/generalpathfinding.png?raw=true" width="303px" height="171px"/>
</p>

A* is a directed algorithm (meaning that it does not blindly search paths) that returns the shortest path between two points (if any). To do it, assesses the best direction to explore, sometimes backtracking to try alternatives.
A* is one of the most used algorithms for Pathfinding since it’s pretty fast, so you may ask why do we need to find optimizations or improvements for A* if it already works in a optimal way finding the shortest path existing. The thing is that pathfinding must be performed mostly in real time and with the less resources and CPU usage as possible, specially if there are many objects that need pathfinding (which is much even for A*). Also we need to improve elements such as memory and computational resources, many pathfindings needed at a little amount of time, dynamic worlds, terrain weights, paths recalculations…

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/Astar%20Trap1.png?raw=true" width="274px" height="274px"/>
</p>

So it’s a matter of efficiency; efficiency helps the program running fast and give more importance to other things, so until know, we make it work, and now, we will make it fast, let’s get into it!

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/iuWB2NM48R2r9q7QhyJfhe-320-80.jpg?raw=true" width="468px" height="344px"/>
</p>

### Before Starting
Before starting, it’s important for you to know how A* works as well as how grid maps are abstracted in graphs to perform pathfinding and how everything is calculated (if you don’t you can check [this](https://www.redblobgames.com/pathfinding/a-star/introduction.html) page or [this](https://www.redblobgames.com/pathfinding/a-star/implementation.html) one about implementing A*). Also, remember that there are, at least, 3 kind of distance calculations that you can use for heuristics (g values) depending on how do you want pathfinding to work in your game. They are:

  * Manhattan Distance for for 4 or 6 directions movement →  d = Cl * (|dx| + |dy|)
  * Diagonal Movement (8 directions): Chebyshev Distance → d = Cd * max(|dx|, |dy|) for Cd = Cl or Octile Distance for Cd = Cl * sqrt(2) → d = Cl * (dx+dy) + (Cd - 2Cl) * min(dx, dy)
  * Euclidean Distance for straight line (any angle/direction) movement, a more expensive calculation and descaling (of g and h) problems →  d = Cl * sqrt(2*dx*dy)
  
Where Cl is the linear cost (horizontal/vertical) of moving, Cd the diagonal one, dx = x-x0 and dy = y -y0. I'm saying this because you can play with heuristics to change the types of paths you get.

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

## A* First Improvements, Generalities and Context
### General Improvements and Heuristics Changes
In this section, I will talk about general A* improvements that can speed up the algorithm. I recommend (if they are used) to mix them with some algorithm explained downwards to a better improvement.

#### Beam Search
#### Bidirectional Search
#### Dynamic Weighting
#### Iterative Deepening (IDA*)
#### Building This Section

### Paths Recalculations - Incremental Searches
#### Fringe Saving A*
#### Generalized Adaptive A* (GAA*) - Initial Approach to Moving Targets
#### Dynamic A* (D*) and Lifelong Planning A* (LPA*)
##### LPA* and D* ’s son: A love story

### Angled Pathfinding
#### Field D*
#### Theta*
#### Incremental Phi*

## Nowadays - Hierarchies and other Games
### Hierarchical Pathfinding (HPA*)
#### Improving HPA* - Partial Refinement A* (PRA*)
### Other Games’ Approaches
##### Professor Lupo - Thankful Mention
#### Navigation Meshes and Hierarchy
##### Starcraft II
##### Dragon Age Origins
##### Heroes on the Move
##### Supernauts
#### Hierarchical Pathfinding in Other Games
##### Castle Story
##### KillZone 2
##### Company of Heroes and Dawn of War 2

## My Approach - Killing Path Symmetries
## Final Thoughts and Recommendations
## Links to Additional Information
