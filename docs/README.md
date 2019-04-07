# A* Optimizations and Improvements
###### Research Work by Lucho Suaya – Universitat Politècnica de Catalunya

## Header
I am Lucho Suaya, a student of the Bahcelor’s Degree in Video Games by [UPC](https://www.upc.edu/ca) at [CITM](https://www.citm.upc.edu/ing/estudis/graus-videojocs/). This content is generated for the second year’s subject Project II, under the supervisión of lecturers [Ricard Pillosu](https://es.linkedin.com/in/ricardpillosu) and [Marc Garrigó](https://www.linkedin.com/in/mgarrigo/).

## Index

## Research Organization
As you may see in the index, this page has many sections and many information. It’s possible that you are not interested in everything that is explained here, so to ease your accessibility, let me explain how the research is organized (apart of introduction part).

   1. First of all, in [A* First Improvements, Generalities and Context](#a-first-improvements-generalities-and-context), I explain the main and minor optimizations for A* and I introduce the concept of Incremental Pathfinding as well as explaining a bit the context in which pathfinding evolved. Finally, I go a bit over angled pathfinding.
  
   2. In the second part, [Nowadays - Hierarchies and other Games](#nowadays---hierarchies-and-other-games) I start with the Hierarchical Pathfinding approach to improve A*, which is very used and common nowadays, and for that reason, I put many examples of other games using it (in fact, it’s difficult to find big games that do not use it).
  
   3. In the 3rd and final part, I introduce and develop [My Approach - Killing Path Symmetries](#my-approach---killing-path-symmetries), to improve A*, which is a way to do it developed by an investigator called Daniel Harabor and makes A* way faster. Also, other improvements are explained since JPS might not be the best take for all games, so to have references if it’s your case (although they are not developed as JPS, I put links to works that show how to do it).
  
Then I conclude this research with additional information and final thoughts.

# Introduction to Problem
As you may deduce, in many games, there is a need to find paths from a location to another, for example, to give a path for a unit to move. To do this, we build algorithms in order to find these paths in an automatic manner. In this research, we will talk about one specific algorithm, which I will suppose you already know, the A*, in order to find optimizations and improvements for it.
![]()
<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/generalpathfinding.png?raw=true" width="605px" height="342px"/>
</p>

A* is a directed algorithm (meaning that it does not blindly search paths) that returns the shortest path between two points (if any). To do it, assesses the best direction to explore, sometimes backtracking to try alternatives.
A* is one of the most used algorithms for Pathfinding since it’s pretty fast, so you may ask why do we need to find optimizations or improvements for A* if it already works in a optimal way finding the shortest path existing. The thing is that pathfinding must be performed mostly in real time and with the less resources and CPU usage as possible, specially if there are many objects that need pathfinding (which is much even for A*). Also we need to improve elements such as memory and computational resources, many pathfindings needed at a little amount of time, dynamic worlds, terrain weights, paths recalculations…

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/Astar%20Trap1.png?raw=true" width="249px" height="249px"/>
</p>

So it’s a matter of efficiency; efficiency helps the program running fast and give more importance to other things, so until know, we make it work, and now, we will make it fast, let’s get into it!

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/iuWB2NM48R2r9q7QhyJfhe-320-80.jpg?raw=true" width="213px" height="156px"/>
</p>

# A* First Improvements, Generalities and Context
# Nowadays - Hierarchies and other Games
# My Approach - Killing Path Symmetries
