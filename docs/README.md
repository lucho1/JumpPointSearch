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
The next is an image of a comparison between A* (in Red) and Lazy Theta* (in Blue, a Theta* improvement):
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/thetavsa.png?raw=true" width="217px" height="216px"/>
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
On top of the previous researches and especially on top of everything explained in the previous sections, pathfinding started to have different directions. What it seems to be widely used by other reference videogames is the map abstraction into a hierarchy with different levels representing the tiles of the lower levels (that is, dividing the map into areas that, each time with bigger tiles, represent the map itself, like a Quadtree).
Before seeing it, let’s see the different types of map representations or how to represent the map in a graph.

### Hierarchical Pathfinding (HPA*)
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/hierarchy.jpg?raw=true" width="214px" height="143px"/>
</p>

Pathfinding with a large number of units on a large graph, all with different starting locations and potentially different destinations can be tricky and time and performance consuming if there is not a good strategy. Hierarchical pathfinding might solve this problem when needing the previous features. It breaks the graph into a hierarchy and, inside each hierarchy levels, into sectors (called clusters) and performs pathfinding first in the higher levels, and then, in the lower levels that these high-level paths touch. This is that, on a high level layer, a path is planned and then, a second one within clusters of lower level and so. HPA* is used to quickly find an optimal path for many units faster than running A* individually. The number of nodes to check out is lower (since we do smaller searches) and the performance is better. 

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/hpa/grid-hierarchy.png?raw=true" width="268px" height="134px"/>
</p>
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/hierarchyPF1.png?raw=true" width="128px" height="116px"/>
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/hierarchyPF2.png?raw=true" width="146px" height="132px"/>
</p>
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/hierarchyPF3.png?raw=true" width="268px" height="220px"/>
</p>

The hierarchical pathfinding addresses, mainly, three issues (the next parts are very well explained in this [AI Game Dev article](http://aigamedev.com/open/review/near-optimal-hierarchical-pathfinding/), which also includes the HPA* paper to download and a evaluation of the algorithm):

  1. Dynamic environment and paths changings
  2. A* ’s slow performance in big maps
  3. Profiting Previous searches (Incremental Search)

And to tackle these, it uses a combination of grid preprocessing to have a high-level graph (with many levels as necessary) using cluster algorithms like Quadtrees, path-planning within the hierarchy (1st at a higher level, then recursively at lower ones) and path re-using.

As you will see, many of the nowadays game uses this approach towards pathfinding. For that reason, you might understand better HPA*, by can check out the next section of this page explaining approaches by other games, especially the ones talking about Castle Story, Heroes on the Move and Killzone.

#### Improving HPA* - Partial Refinement A* (PRA*)
HPA* also has some improvements. One of them is Partial Refinement A* (PRA*), that combines the hierarchical map abstraction like HPA* but also connects the different levels of the hierarchy (keeping the current nodes’ connectivity inside a same level), forming kind of pyramids. Then, to find a path, a level of the hierarchy is chosen and search for nodes representing the origin and destination nodes of that path.
So, at the higher level chosen, PRA* uses A* to find a path and then projects the first steps down to the next hierarchy levels until reaching the final one (the one in which we need the path to work), where the final path is refined, meaning that is definitively done by considering a corridor around the previous projected path. However, PRA* performance is similar to HPA*.

[This paper](https://drive.google.com/open?id=1UsIyOZQZwcV07r3xKoZMbOmIl2Sdp24h) explains PRA* better and [this one](https://drive.google.com/open?id=17HYI_ZHk0RQ_LV1bzESXIis1mQaEZ85r) purposes an improvement on it.

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

### Other Games’ Approaches
Until here, we have been looking different ways to improve A* ’s performance. Anyway, we should also see how other different games try to overcome the problem to take them as a reference (since they are the ones that have people investigating for their games to work). First of all, to have the first clue, mention that [this]() project is a package for unity that provides an improved A* version and supports different graph set ups (navigation meshes, waypoints…) and even ways to automate them. It’s used by games like [Kim](https://store.steampowered.com/app/433400/Kim/), [Folk Tale](http://www.gamesfoundry.com/), [Divide](http://www.explodingtuba.com/), [CubeMen](http://cubementd.com/) or [Dark Frontier](https://www.youtube.com/watch?v=tOc-xdtufmg).

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/unity.jpg?raw=true" width="285px" height="154px"/>
</p>

I would like to mention that we have tried (with time enough, like a month or two ago) to contact different studios and companies to ask them how they work towards pathfinding, but only two answered. I would like to thank them, one is [Beautifun Games](https://beautifungames.com/) (whose we will talk now) and the other is [Yatch Club Games](https://yachtclubgames.com/), developers of games like [Shovel Knight](https://yachtclubgames.com/shovel-knight/) or [Cyber Shadow](https://yachtclubgames.com/cyber-shadow/). I won’t talk about them because they answered that they do not use pathfinding in their games since they are simple enough.

Without further delay, let’s keep moving!

###### Professor Lupo - Thankful Mention
[Professor Lupo](http://www.professorlupo.com/), from Beautifun Games, uses [Bresenham](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) algorithm for straight lines. We won’t go deeper into it, but if you are interested, [here](https://deepnight.net/tutorials/bresenham-magic-raycasting-line-of-sight-pathfinding/) there is a blog explaining it with code implementation and a check for obstacles blocking the sight between monsters and the player (case in which pathfinding should not be done, and therefore skipping useless calls) and some other features.
[Red Blob Games](https://www.redblobgames.com/) has also a [page](https://www.redblobgames.com/grids/line-drawing.html) in which explains line drawing with Bresenham and the concept of linear interpolation.
Also, you can check a fragment of [this book](https://books.google.es/books?id=Sz-Sqvm-hSYC&pg=PA12&lpg=PA12&dq=bresenham+for+pathfinding&source=bl&ots=vOkq_j6DgK&sig=ACfU3U1yUTEFxRg6QMwJqfkhpZaXGJFF1g&hl=es&sa=X&ved=2ahUKEwiCserggZHhAhUC2uAKHXHlBEAQ6AEwCXoECAgQAQ#v=onepage&q=bresenham%20for%20pathfinding&f=false) (AI For Game Devs), which explains more deeper the subjects mentioned.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/professor-lupo-and-his-horrible-pets-2017118121656_1.jpg?raw=true" width="300px" height="158px"/>
</p>

#### Navigation Meshes and Hierarchy
###### Starcraft II
The developers of [Starcraft II](https://starcraft2.com/es-es/) had to lead with hundreds of units moving over a map with different terrains while improving the pathfinding of previous titles (this means, better IA with better and realistic paths).
[James Anhalt](https://www.mobygames.com/developer/sheet/view/developerId,20733/), Lead Software Engineer on Game Systems of [Blizzard](https://www.blizzard.com/es-es/), explained at GDC 2011 that to attack the problem, they represented the world by using a triangulated Navigation Mesh** over which A* is run.
As they needed to save a lot of memory, and to make it more faster, they statically allocated everything and condensed the structures into 16 bytes per face of triangle and 4 bytes per vertex, which allowed to have 64000 faces and 32000 vertices.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/starcraftTriangulation.png?raw=true" width="323px" height="182px"/>
</p>

I won’t talk more about Starcraft II pathfinding since it’s not in the direction of this research, but if you want to know more, you can go to that [2011 GDC talk](https://www.gdcvault.com/play/1014514/AI-Navigation-It-s-Not) in which he explains it deeper (minutes 3 to 20), which is the same talk in which the orators explain the pathfinding for Heroes on the Move and Dragon Age Origin. Also, check the section 2.4 (Triangulation Based Pathfinding) of [this Pathfinding book](http://drops.dagstuhl.de/opus/volltexte/2013/4333/pdf/4.pdf) (pages 24-25) that explains a bit about Pathfinding with triangulated map representations.

> ** *Specifically, [Constrained Delaunay Triangulation](https://en.wikipedia.org/wiki/Constrained_Delaunay_triangulation), which is a generalization of the [Delaunay Triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation) that not always accomplishes a rule called the Delaunay Condition that states that the circle circumscribed of each triangle dividing, in this case the map, cannot contain any vertex of other triangle.
A [Triangulation](https://en.wikipedia.org/wiki/Triangulation_(geometry)) is the division of the map into triangles, for the people who does not know.*

###### Dragon Age Origins
The case of [Dragon Age Origins](https://www.ea.com/es-es/games/dragon-age/dragon-age-origins) is similar to the ones of its same talk in [GDC 2011 talk](https://www.gdcvault.com/play/1014514/AI-Navigation-It-s-Not) (minute 36 to 54), Starcraft II and Heroes on the Move, and as they were before, [Nathan Sturtevant](https://www.cs.du.edu/~sturtevant/) (consultant for [BioWare](http://www.bioware.com/) that implemented pathfinding engine) do not explain a lot regarding to pathfinding itself since it seems pretty similar to the previous talks, so it focuses more on path smoothing, quality and trap avoidance (how to prevent pathfinding to make weird things and make it look good).
As told, Dragon Age’s approach is similar to the previous: they hierarchize the map in a low level graph (represented by grids) and a higher level to make the world movement (represented by a kind of a Navigation Mesh). Also, they divide the whole map in sectors and sub-sectors (called regions).
Their goals towards pathfinding were to achieve a fast planner while planning long paths and being memory efficient. The next are images of a prototype for Dragon Age with a low level grid representation (red) and the second image has, in yellow, the representation of the high level grid.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/DAOAbstractGraph.png?raw=true" width="323px" height="182px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/DAOLevelsAbs.png?raw=true" width="323px" height="182px"/>
</p>

Just for curiosity, he mentions that they use octile heuristics. Also (and because of curiosity too) it mentions that Google can perform fast pathfinding because they rely a lot on roads properties (highways are the faster way to get somewhere).

###### Heroes on the Move
The Playstation’s [Heroes on the Move](https://es.wikipedia.org/wiki/PlayStation_Move_Heroes) developers had to come up with a solution that allowed runtime planning of pathfinding while spending no more than the 50% of the time on navigation, so they did something similar to Starcraft II but they also applied hierarchy levels and calculate paths in them to smooth them later. First of all, they triangulated a Navigation Mesh.

With this, they have a high level (with two nodes per cell):

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/NavMesh.png?raw=true" width="335px" height="189px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/NavMeshintoClusters.png?raw=true" width="335px" height="189px"/>
</p>

And, when they wanted to find a path, they found a solution first in the abstract graph:

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/NodesofNavMesh.png?raw=true" width="258px" height="145px"/>
</p>

And then they went, in the abstract graph, area per area building a path. This means that, once they have an abstract solution, first pick an area and do the path inside and then the other and so while the game keeps running. Like this:

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/AbstractGraphSol.png?raw=true" width="258px" height="145px"/>
</p>

And when the element moving arrives to the “portal” (the frontier between areas), they calculate the path to the next area. This allows what they seek: to pathfind in runtime and keeping it fast (not spending much time).
To know the place where to cross between lines, they just calculated the nearest point to the moving element.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/HotM/LowLevelSolution.png?raw=true" width="258px" height="145px"/>
</p>

This pathfinding way is also explained in the same [GDC 2011 talk](https://www.gdcvault.com/play/1014514/AI-Navigation-It-s-Not) than Starcraft II and Dragon Age Origin (from minute 20 to 36).

###### Supernauts
Harri Hatinen, Lead Programmer of [Grand Cru](http://grandcrugames.com/), explained in the GDC China 2014 that they abstracted the map to a [Navigation Mesh](https://en.wikipedia.org/wiki/Navigation_mesh) and mixed with hierarchy representation for [Supernauts](http://supernauts.com/). I could recover the [presentation](https://drive.google.com/file/d/1HIj5jPo2WcK7q9KyQ6ADoWFakRCGPbd-/view?usp=sharing) in which explains how they handle the navigation map, how they make it work (taking into account the frontier between nodes and its longitude, How the IA knows where to pass by in a large frontier between nodes?) and how they update it each time that the map changes.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/supernauts.jpg?raw=true" width="341px" height="256px"/>
</p>

There is a [video](https://www.gdcvault.com/play/1021705/Advanced-Real-time-Path-Find) of that presentation, but it’s a little annoying to hear a chinese voice over the Harri Hatinen’s viking’s voice (and the video quality is a bit low, but with the diapositives it can be understood).

#### Hierarchical Pathfinding
###### Castle Story
Another interesting talk, this case in the GDC 2018, was the one by Alain Benoit, Lead Programmer and CTO (Chief Technology Officer) of [Sauropod Studio](https://www.sauropodstudio.com/english-1). I could only find the [diapositives](https://drive.google.com/open?id=1oH2uebOgW39KZxkZp4Z6xxiqnfAX9yS_)’ presentation because to see the video you must be a GDC member (which I’m not, I’m a student and sadly I don’t have 550$ each year to invest in GDC), but in case you are, I leave you the search of that presentation [here](https://www.gdcvault.com/search.php#&category=free&firstfocus=&keyword=Benoit+Alain&conference_id=).
Anyway, with the diapositives it’s pretty understandable that for their game, [Castle Story](http://www.castlestory.net/), they decided to use Hierarchical Pathfinding to fulfill their needs to have many agents moving, dynamic obstacles, stairs and blockages, deformable terrain and buildable blocks in a large scale map.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/Astar%20Trap2.png?raw=true" width="326px" height="242px"/>
</p>

The truth is that this way of solving pathfinding problems is very effective and it solves them in a fast way. So, as told, they us hierarchy to abstract the grid and improve pathfinding efficiency, see it with images:

<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy1.PNG?raw=true" width="641px" height="167px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy2.PNG?raw=true" width="638px" height="163px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy3.PNG?raw=true" width="640px" height="164px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy4.PNG?raw=true" width="642px" height="168px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy5.PNG?raw=true" width="400px" height="139px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy6.PNG?raw=true" width="261px" height="140px"/></p>
<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy7.PNG?raw=true" width="242px" height="282px"/></p>

To find optimum paths, they state a certain chain of rules such as explore lower hierarchy levels as getting closer to the goal, or exploring them if they have dynamic obstacles, not exploring childs if parents already were...

<p align="center"><img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Castle%20Story/hierarcy8.PNG?raw=true" width="435px" height="184px"/></p>

###### KillZone 2
I can’t talk that much about Killzone 2 since the only thing I have is [this](https://drive.google.com/file/d/1GlsWO-dw_8zj3AENfcwjmSQysAmHZIH9/view) power point from the Game AI Conference celebrated in Paris in 2009.
From the diapositives I could translate and extract that they use kind of a mix between waypoints and hierarchy (a set of areas created as groups of waypoints):

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Killzone/killzone1.PNG?raw=true" width="305px" height="234px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Killzone/killzone2.PNG?raw=true" width="303px" height="232px"/>
</p>
<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Killzone/killzone3.PNG?raw=true" width="304px" height="231px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Killzone/killzone4.PNG?raw=true" width="304px" height="228px"/>
</p>

This is for supporting strategic decision making algorithms. They do a high-level graph based on the low-level waypoint network and use an automatic area generation algorithm.
So, they have dynamic information over the strategic graph for many decisions (hide, defend, regroup…) and to store some influence information based on faction controls of the area, and they calculate it based on all bots, turrets… They put an example of this.

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/Killzone/killzone7.PNG?raw=true" width="303px" height="228px"/>
</p>

Over this, they work with pathfinding. Think that the game needs to make strategic decisions taking into account the previous strategic and influence graphs.
So, they use a single-source pathfinder to calculate distances to a point, the algorithm combines Dijkstra and [Bellman-Ford-Moore](https://en.wikipedia.org/wiki/Bellman%E2%80%93Ford_algorithm) (paths from a single-source from all other vertices in a weighted directed graph, slower than Dijkstra but more versatile since handles nodes with negative numbers), but with some tricks to make it more efficient and improving the distance estimates (the heuristics) when multiple updates.
Now from here, each squad of units has an own pathfinder which finds a path between waypoints in the selected areas.

Again, this is a translation with some interpretations of the powerpoint linked above, I’m not 100% sure on how it actually works since I can’t go deeply on it with only this presentation, so I might be wrong in something.

###### Company of Heroes and Dawn of War 2
The [AI Game Dev](http://aigamedev.com/) platform (sadly seems closed by now), interviewed Chris Jurney, which worked as Senior Programmer in [Relic Entertainment](http://aigamedev.com/) and at [Kaos Studios](http://aigamedev.com/). It was part of the development of Company of Heroes and Dawn of War 2, games for which he was interviewed in the interview that I’m talking about. You can see the transcription in [PDF](http://aigamedev.com/) and the [MP3](http://aigamedev.com/) recording.

In there, he explains that the pathfinding in Company of Heroes worked as well with Hierarchical Pathfinding in a map of thousand of meters (so thousands of cells) in which the worst case was 1000x250 (250 000 cells). This was very complicated to implement (especially for map changes and updates), so he says that HPA* can be a good complexity-decrementer (event thought it confesses that he did not tried).

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/dawnofwar.jpg?raw=true" width="313px" height="176px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/other%20games/companyofheroes.jpg?raw=true" width="250px" height="188px"/>
</p>

With hierarchical pathfinding, he says that each unit can recalculate behaviour each 3/10 to half a second to follow a leader that actually knows the whole picture and re-evaluates the route every second.
In the GDC China 2007, this man, along with Shelby Hubick give a presentation about AI in Destructive environments in Company of Heroes, in case you are interested in it. This is the [Audio](https://drive.google.com/open?id=1r9MsnRnyEZ6S28W-YduDSiUJf2pat2Wc) (again, couldn’t find a video, sorry).
In fact, I recently discovered that in [this](http://aigamedev.com/open/tutorials/clearance-based-pathfinding/) article of AI Game Dev, is said that Chris Jurney at a that GDC 2007 talk, explains that they use a similar method to HAA* (explained downwards) with the usage of a variant of [Brushfire](http://roboscience.org/book/html/Planning/Brushfire.html) algorithm to put numeric values to the tiles of the map according their nearness to an obstacle and, therefore, calculating if the size of a troop make it able to pass through an area. The reason why this is not very explained here is because is not very shared in the talk and the article about it in the AI Game Dev page is premium (and I cannot have access). Anyway, you have a similar approach detailed downwards if you are interested (HAA* section, link above in this paragraph).

***
> *Many information? Looking for other section? Go back to [Index](#index)*
***

## My Approach - Killing Path Symmetries
In this section I will explain my approach to improve as much as possible the A* pathfinding taking into account the projects of video games creation that we are developing for a University’s subject, in which we will need a pathfinding fast as possible, but it might serve for any reader or other projects, too.
I will deeply explain Jump Point Search algorithm because is the one that best fits into my team’s project, but I will also leave an explanation of two other systems called Rectangular Symmetry Reduction and Hierarchical Annotated Pathfinding, just in case that Jump Point Search does not fit to your requirements. Anyway, I won’t explain deeply its development (unlikely JPS) because of space and time matters, but I will let you some documentation so you can access to the information needed to implement it in case you need it.

Before going into it, let me explain what a path symmetry is. As you may guess, from a starting point, there might be many ways to reach a destination, this means that for the same start/destination, there can be many paths. Well, symmetric paths are these kind of paths in which the only difference is the places they pass by.
The formal definition is: “Two paths in a grid are symmetric if they share the same start and end point and one can be derives from the other by swapping the order of the constituent vectors”. They would look like:

<p align="center">
 <img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/jps/pathSymmetries.PNG?raw=true" width="155px" height="155px"/>
<img src="https://raw.githubusercontent.com/lucho1/JumpPointSearch/master/docs/Images/jps/pathSymmetries2.PNG?raw=true" width="175px" height="153px"/>
</p>

A* explores all the paths even if they are symmetric, so it makes lots of useless efforts and searches traduced in decreasing performance, so many approaches to improve A* focus on killing or avoiding path symmetries, having great results. In this page, we will see two of them, but we will focus especially on one, explaining its development, an algorithm A*-based called Jump Point Search. [This](https://harablog.wordpress.com/2011/08/26/fast-pathfinding-via-symmetry-breaking/) page contains more information about this (but I think it was clearly explained and does not needs further expanding, I place the link here to make you know that the information comes from there).

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
