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
     * [Hierarchical Pathfinding (HPA*)](#hierarchical-pathfinding-hpa)
       * [Improving HPA* - Partial Refinement A* (PRA*)](#improving-hpa---partial-refinement-a-pra)
     * [Other Games’ Approaches](#other-games-approaches)  
         * [Professor Lupo - Thankful Mention](#professor-lupo---thankful-mention)     
       * [Navigation Meshes and Hierarchy](#navigation-meshes-and-hierarchy)
         * [Starcraft II](#starcraft-ii)
         * [Dragon Age Origins](#dragon-age-origins)
         * [Heroes on the Move](#heroes-on-the-move)
         * [Supernauts](#supernauts)
       * [Hierarchical Pathfinding](#hierarchical-pathfinding)       
         * [Castle Story](#castle-story)
         * [KillZone 2](#killzone-2)
         * [Company of Heroes and Dawn of War 2](#company-of-heroes-and-dawn-of-war-2)
  5. [My Approach - Killing Path Symmetries](#my-approach---killing-path-symmetries)
       * [Jump Point Search (JPS)](#jump-point-search-jps)  
         * [Introduction](#introduction)
         * [Pruning](#pruning)
           * [Neighbour Pruning Rules](#neighbour-pruning-rules)
         * [C/C++ Implementation](#cc-implementation)
           * [Step by Step Implementation - Do it Yourself Exercise](#step-by-step-implementation---do-it-yourself-exercise)      
           * [Exercise Solutions](#exercise-solutions)           
           * [Performance](#performance)         
           * [More Information on JPS and Sources](#more-information-on-jps-and-sources) 
       * [Other Improvements](#other-improvements)
         * [Rectangular Symmetry Reduction (RSR)](#rectangular-symmetry-reduction-rsr)
         * [Hierarchical Annotated A* (HAA*)](#hierarchical-annotated-a-haa)
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
Beam Search sets a limit on the size of OPEN list and, if reached, the elements with less priority are deleted (or which is the same: the elements with less chance of giving a good path). Only the most promising nodes are retained for further branching (reducing memory requirements).

#### Bidirectional Search
Bidirectional Search is to start two searches in parallel, one from start to goal (being last node found X) and other from goal to start (being last node found Y). When they meet, the search is ended. Instead of doing the A* ’s heuristic, it calculates it as: F = G(start, X) + H (X, Y) + G(Y, goal).

#### Dynamic Weighting
At the beginning of the search, it’s assumed that is more important to get to the destination rather fast than better, so it changes heuristics to F = G + H * W. This W is a weight associated to the heuristic which goes decreasing as getting closer to the goal, so it goes decreasing the importance of the heuristics and increases the importance of the actual cost of the path, making it faster at the begin and better at the end.

#### Iterative Deepening (IDA*)
[IDA*](https://es.wikipedia.org/wiki/IDA*) is an algorithm that can find the shortest path in a weighted graph. It uses A*’s heuristic to calculate the heuristic of getting to the goal node but, since it’s a depth-first search algorithm, it uses less memory than A*. Even though, it explores the most promising nodes so it does not go to the same depth always (meaning that the time spent is worth). It might explore the same node many times.
So, basically, it starts by performing a depth-first search and stops when the heuristics (F = H + G) exceeds a determined value, so if the value is too large, it won’t be considered. This value starts as an estimate initially and it goes increasing at each iteration.

IDA* works for memory constrained problems. While A* keeps a queue of nodes, IDA* do not remember them (excepting the ones in current path), so the memory needed is linear and lower than in A*. A good thing of IDA* is that doesn’t spend memory on lists since it goes in-depth, but it has some disadvantages that the usage of a base algorithm like [Fringe Search](https://en.wikipedia.org/wiki/Fringe_search) can solve (at least partially).
[This](https://drive.google.com/open?id=1Yu3bnZXCnRsxiuk6l-ZbHEQbF7mGe8a2) paper explains it deeper.

#### Building This Section
To build this section, apart of the links used and placed over the text above, I have used fragments of [this](https://drive.google.com/open?id=1D_NkL1co6TvEbcS5PyoceycbSPr59IDu) thesis that explains, among others, interesting things on Heuristics, IDA*, A* and Fringe Search.
Also, I have used a part of [this](http://theory.stanford.edu/~amitp/GameProgramming/Variations.html) page in [Amit’s Thoughts on Pathfinding](http://theory.stanford.edu/~amitp/GameProgramming/).

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

### Paths Recalculations - Incremental Searches
In this section I will talk about a way to improve and speed up A* through [incremental searches](https://drive.google.com/open?id=1tmqB_ooKRxiBzbYO6aB7ptmx3B62rwTk) which uses information from previous searches to have a basis over which to build the new searches and, therefore, spend less time (speed up searches for sequences of similar problems by using experience from previous problems).

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/incremental.jpg?raw=true" width="172px" height="138px"/>
</p>

Incremental searches were developed for mobile robots and autonomous vehicle navigation, so it’s a quite deep sector. They tend to have more efficient algorithms than A* (and based on it), but, as they were thought to robotics, they support one only unit in movement, so they might be inefficient for many pathfindings in short times, like the ones we need in games, but it’s good to know them to have a deeper vision on pathfinding.

#### Fringe Saving A*
[Fringe Saving A*](https://drive.google.com/open?id=1ejkyIuEKYb5_12k2F9QNcLZrgsZ212F1) launches an A* search and then, if detects a map change, restores the first A* search until the point in which map has changed. Then it begins an A* search from there, instead of doing it from scratch.

#### Generalized Adaptive A* (GAA*) - Initial Approach to Moving Targets
This variant of A* can handle moving target points. When there is a moving point, the H variable of the heuristics change. If the target is constantly moving, these H values might become inconsistent, so Generalized Adaptive A* (GAA*) updates these H values using information of previous searches and keeps them consistent.

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/movingtargetGIF.gif?raw=true" width="147px" height="147px"/>
</p>

This allows to find shortest paths in state spaces where the action costs can increase over time since consistent h-values remain consistent after action cost increases. It’s easy to implement and understand and better explained in [this article](https://drive.google.com/open?id=1e4gbDqxuCwCO9b-5i8fIykFHqmW92qTK).

#### Dynamic A* (D*) and Lifelong Planning A* (LPA*)
<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/D.jpg?raw=true" width="125px" height="125px"/>
</p>

[Dynamic A*](https://en.wikipedia.org/wiki/D*) (D*) was developed for robotics (mobile robot and autonomous vehicle navigation) to make robots navigate with the shortest path possible and towards a goal in an unknown terrain based on assumptions made (such as no obstacles in the way).

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/bostondynamics-640x353.jpg?raw=true" width="160px" height="88px"/>
</p>

If the robot detects new map informations (for instance, previously unknown obstacles), adds the information to its previous assumptions and, if it’s necessary, it replans a new path. Doing this ensures to have more speed than restarting A*, so D* is for cases in which the complete information is not available, cases in which  A* can make mistakes, but D* solves them quickly (making it fit to dynamic obstacles) and allowing fast-replanning.
But nevertheless, even though being similar to A*, is more complex and has been obsoleted by a new version which is simpler.

###### LPA* and D* ’s son: A love story
<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/lpa.png?raw=true" width="57px" height="57px"/>
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/love.jpg?raw=true" width="93px" height="59px"/>
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/D.jpg?raw=true" width="62px" height="62px"/>
</p>

Parallely, there is an (obsoleted) algorithm called DynamicSWSF-FP that stored the distance from every node to the destination node. It had a big initial setup when calling it, but after graph changes, it updated ONLY the nodes whose distances had changed.
This is important for [Incremental A*](https://drive.google.com/open?id=1tHE54ptXXKkmyM84ALw130HJtO4LNvne), also called Lifelong Planning A* (LPA*) which is a combination between DynamicSWSF-FP and A*. In the core, is the same than A* but when the graph changes, the later searches for the same start/finish pairs uses the information of previous searches to reduce the number of nodes to look at, so LPA* is mainly used when the costs of the nodes changes because with A*, the path can be annulled by them (which means that has to be restarted). LPA* can re-use the previous computations to do a new path, which saves time (but it consumes memory).
Although, LPA* has a problem: it finds the best path from the same start to the same finish, but is not very used if the start point is moving or changing (such as units movement), since it works with pairs of the same coordinates. Also, another problem is that both D* and LPA* need lots of space (because, to understand it, you run A* and keep its information) to, if the map changes, decide if adjusting the path fastly. This means that, in a game with many units moving, it’s not practical to use these algorithms because it’s not just the memory used, but the fact that they been designed for robots, which is one only unit moving; if you use them for many units, it stops being better than A*.

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/WALLE.jpeg?raw=true" width="228px" height="152px"/>
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/robotPATH.png?raw=true" width="162px" height="182px"/>
</p>

So, to fix LPA* ’s heart, it appeared [D* Lite](https://drive.google.com/open?id=1_QszpFqF8jxi3zLHE1_JJQKlAd3cQVPU) (not based on D*), mentioned before as it obsoleted D*. Generally, D* and D* Lite have the same behaviour but D* Lite uses LPA* to recalculate map information (LPA* is better on doing so). Also, is simpler to implement and to understand than D* and always runs at least as fast as D*.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/DLite.png?raw=true" width="137px" height="135px"/>
</p>

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

### Angled Pathfinding
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/angle.png?raw=true" width="162px" height="102px"/>
</p>

[Angled pathfinding](https://en.wikipedia.org/wiki/Any-angle_path_planning) is very useful to build more realistic paths and also to improve A* performance. Algorithms allowing this do not only explore from node to node at 45º or 90º like usual, but at any angle thanks to [visibility](https://www.redblobgames.com/articles/visibility/). This means that, instead of just going node per node, if there is a farther node that is visible from the current node (no obstacles in the middle), it will set the parent of the visible node to the current node no matter the angle between both nodes. As said, this makes path to be more realistic while also improving the base A* performance (so we have a win-win!).

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/angledpath.png?raw=true" width="332px" height="138px"/>
</p>

For a comparison and explanation between different angled pathfinding algorithm, check out [this paper](https://drive.google.com/open?id=1O7HlD0lUHE7M5B2ruPDhUol4wgrRnG7M).

#### Field D*
Field D* is a variant of D* Lite that does not constraints to a grid, it gives the best path moving along any angle adding smooth to the path by using interpolation (which complicates it a bit). It was used for a Mars Rovers, see [this paper](https://drive.google.com/open?id=1GbV9tLGrmLRy2EqLdXikn8gY7Ko4xnAA) for more information.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/Mars1.jpeg?raw=true" width="192px" height="192px"/>
</p>

#### Theta*
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/thetavsa.png?raw=true" width="167px" height="166px"/>
</p>

Theta* is an A* and D* variant (like Field D*) but it doesn’t has fast-replanning capabilities. It runs on square grids and it finds shortest paths that do not strictly follow the grid by pointing to adjacent ancestors, if there is a line of sight towards that node, it can save it as a parent node, skipping the nodes in-between (what is called visibility).
See this [AI Game Dev article](http://aigamedev.com/open/tutorials/theta-star-any-angle-paths/) for further information, is well explained there. Also you can see this [paper](https://drive.google.com/open?id=1hMGIUlks5_YRiU9zSVlU0H2R-VNUp-so) in which there’s a longer and deeper explanation on angled pathfinding and also, on Block A*, which is a version of Theta* that is faster because it uses a hierarchical approach ([this article](https://drive.google.com/open?id=1jbFISJ4SRsnVN2R8UWHvOxKWuXFQgQvY) gets even more information on Block A*). 

Appart of Block A*, there’s also another faster version for Theta* called Lazy Theta*. You can check this AI Game Dev article for more information on it, basically, in four more lines of code, it makes Theta* faster by performing less line of sight checks.

<p align="center">
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/thetavslazytheta.png?raw=true" width="291px" height="169px"/>
   <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/thetavslazytheta2.png?raw=true" width="350px" height="158px"/>
</p>

A good advantage of these algorithms is that they are pretty easy to understand and implement.

#### Incremental Phi*
To finish with angled pathfinding, just mention Incremental Phi*. It mixes Theta* and Field D*, by making an incremental version of Theta* (so, allowing fast-replanning). It’s useful for dynamic environments. [This paper](https://drive.google.com/open?id=1IlMBIdmF6ARN_VX7yaxn9NK5Z0mz-IlN) gets deep into it.

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

## Nowadays - Hierarchies and other Games
### Hierarchical Pathfinding (HPA*)
#### Improving HPA* - Partial Refinement A* (PRA*)

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

### Other Games’ Approaches
###### Professor Lupo - Thankful Mention
#### Navigation Meshes and Hierarchy
###### Starcraft II
###### Dragon Age Origins
###### Heroes on the Move
###### Supernauts
#### Hierarchical Pathfinding
###### Castle Story
###### KillZone 2
###### Company of Heroes and Dawn of War 2

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

## My Approach - Killing Path Symmetries
### Jump Point Search (JPS)
#### Introduction

#### Pruning
###### Neighbour Pruning Rules

#### C/C++ Implementation
###### Step by Step Implementation - Do it Yourself Exercise
###### Exercise Solutions
###### Performance
###### More Information on JPS and Sources

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

### Other Improvements
#### Rectangular Symmetry Reduction (RSR)
#### Hierarchical Annotated A* (HAA*)

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

## Final Thoughts and Recommendations
## Links to Additional Information

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***
