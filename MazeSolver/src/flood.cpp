#include "flood.h"
#include "common.h"
#include "wall.h"
#include "line.h"
#include <Arduino.h>

// ---------------- Configuration ----------------
static int MAZE_N = 9;      // NxN logical maze (use 9 as you specified)
static int CELL_CM = 25;    // cell size in cm
static const int US_FRONT_TH = 10; // cm threshold to treat as wall
static const int US_SIDE_TH  = 10;

static const int INT_MAX = 1000000;

void setMazeSize(int n){ if(n>=3 && n<=15) MAZE_N = n; }
void setCellCm(int cm){ if(cm>0) CELL_CM = cm; }

// ---------------- Maze structures ----------------
struct Cell {
  bool wall[4]; // N,E,S,W
  bool visited;
  bool isGoal;
};

static Cell maze[16][16]; // support up to 16x16; we use MAZE_N
static int distMap[16][16];

enum Dir { NORTH=0, EAST=1, SOUTH=2, WEST=3, UNKNOWN_DIR=-1 };

// Robot logical pose (0..MAZE_N-1)
static int rx, ry;
static Dir rface;

// helper
static inline bool inBounds(int x,int y){ return x>=0 && y>=0 && x<MAZE_N && y<MAZE_N; }
static void dirToDelta(Dir d, int &dx, int &dy) {
  if (d==NORTH){dx=0;dy=1;}
  else if(d==EAST){dx=1;dy=0;}
  else if(d==SOUTH){dx=0;dy=-1;}
  else {dx=-1;dy=0;}
}

// ---------- Debug / status helpers ----------

// Return true if every cell in the logical MAZE_N x MAZE_N grid has been visited.
// NOTE: this checks the whole grid. If you want "all reachable visited", use a different test
// (but with DFS scanning, visited==true for reachable cells; unreachable cells remain false).
static bool isMazeFullyScanned()
{
  for (int x = 0; x < MAZE_N; ++x) {
    for (int y = 0; y < MAZE_N; ++y) {
      if (!maze[x][y].visited) return false;
    }
  }
  return true;
}

// Return whether the given absolute direction (0=N,1=E,2=S,3=W) has a wall in current cell.
// Guarded by inBounds check for safety.
static bool hasWallAbsolute(int absDir)
{
  if (!inBounds(rx, ry)) return true; // treat out-of-bounds as wall
  return maze[rx][ry].wall[absDir];
}

// Get wall presence relative to robot facing:
// rel = 0 -> front, rel = 1 -> right, rel = -1 -> left, rel = 2 or -2 -> back
static bool hasWallRelative(int rel)
{
  int absDir = (int)rface + rel;
  // normalize to 0..3
  absDir %= 4;
  if (absDir < 0) absDir += 4;
  return hasWallAbsolute(absDir);
}

// Convert Dir to textual name
static const char* dirName(Dir d) {
  switch(d) {
    case NORTH: return "NORTH";
    case EAST:  return "EAST";
    case SOUTH: return "SOUTH";
    case WEST:  return "WEST";
    default:    return "UNK";
  }
}

// Print a short status line: logical position, facing, walls (front/left/right), scanned flag
static void printStatus()
{
  // Print position and facing
  Serial.print("POS: ");
  Serial.print(rx); Serial.print(","); Serial.print(ry);
  Serial.print("  F: "); Serial.print(dirName(rface));
  Serial.print("  Walls[F/L/R]: ");

  // front: rel = 0
  Serial.print(hasWallRelative(0) ? "1" : "0"); Serial.print("/");
  // left: rel = -1 (or +3)
  Serial.print(hasWallRelative(-1) ? "1" : "0"); Serial.print("/");
  // right: rel = +1
  Serial.print(hasWallRelative(1) ? "1" : "0");

  // visited & goal
  Serial.print("  Visited: "); Serial.print(maze[rx][ry].visited ? "Y" : "N");
  Serial.print("  GoalHere: "); Serial.print(maze[rx][ry].isGoal ? "Y" : "N");

  // entire maze scanned?
  bool allScanned = isMazeFullyScanned();
  Serial.print("  AllScanned: "); Serial.println(allScanned ? "Y" : "N");
}


// rotate robot physically to face target direction (min turns)
static void rotateToFace(Dir target)
{
  if (target == rface) return;
  int diff = (int)target - (int)rface;
  if (diff <= -3) diff += 4;
  if (diff >= 3) diff -= 4;

  if (diff == 1 || diff == -3) {
    smoothTurnLeft();
  } else if (diff == -1 || diff == 3) {
    smoothTurnRight();
  } else if (abs(diff) == 2) {
    pivot180(PIVOT_180_PWM);
  }
  rface = target;
  delay(80);
}

// Move forward exactly one cell (encoder-driven)
static void moveOneCellForward()
{
  moveForwardWallFollow(CELL_CM, BASE_PWM_STRAIGHT);
  stopMotors();
  // update logical location
  int dx,dy; dirToDelta(rface, dx, dy);
  rx += dx; ry += dy;
  delay(50);
}

// Use IR line sensors to test whether current cell is goal (all white)
static bool detectWhiteGoalAtCurrent()
{
  readSensors();
  for (int i=0;i<8;i++) if (sensorVal[i]) return false;
  return true;
}

// Sense walls around robot using ultrasonics and update maze table
static void senseWallsAtCurrent()
{
  long front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long left  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // front
  if (front>0 && front < US_FRONT_TH) {
    maze[rx][ry].wall[rface] = true;
    int dx,dy; dirToDelta(rface, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(rface+2)%4] = true;
  }
  // left
  Dir leftDir = (Dir)((rface+3)%4);
  if (left>0 && left < US_SIDE_TH) {
    maze[rx][ry].wall[leftDir] = true;
    int dx,dy; dirToDelta(leftDir, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(leftDir+2)%4] = true;
  }
  // right
  Dir rightDir = (Dir)((rface+1)%4);
  if (right>0 && right < US_SIDE_TH) {
    maze[rx][ry].wall[rightDir] = true;
    int dx,dy; dirToDelta(rightDir, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(rightDir+2)%4] = true;
  }
}

// Flood (distance) computation (BFS) from all goal cells
static void computeDistancesToGoals()
{
  // init
  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) distMap[x][y] = INT_MAX;

  // queue
  struct P { int x,y; };
  P q[256]; int qh=0, qt=0;

  // push all goal cells
  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) {
    if (maze[x][y].isGoal) {
      distMap[x][y]=0;
      q[qt++] = {x,y};
    }
  }

  // if no explicit goal found, use centre cell(s)
  if (qh==qt) {
    int mid = MAZE_N/2;
    if (MAZE_N%2==1) { distMap[mid][mid]=0; q[qt++] = {mid,mid}; }
    else {
      int a = mid-1, b = mid;
      distMap[a][a]=0; q[qt++] = {a,a};
      distMap[a][b]=0; q[qt++] = {a,b};
      distMap[b][a]=0; q[qt++] = {b,a};
      distMap[b][b]=0; q[qt++] = {b,b};
    }
  }

  while (qh < qt) {
    P p = q[qh++]; int d = distMap[p.x][p.y];
    for (int dir=0; dir<4; ++dir) {
      if (maze[p.x][p.y].wall[dir]) continue;
      int dx,dy; dirToDelta((Dir)dir, dx, dy);
      int nx=p.x+dx, ny=p.y+dy;
      if (!inBounds(nx,ny)) continue;
      if (distMap[nx][ny] > d + 1) {
        distMap[nx][ny] = d + 1;
        q[qt++] = {nx,ny};
      }
    }
  }
}

// Pick neighbor with smallest distMap value (prefer forward, right, left, back)
static Dir pickBestNeighborToReduceDistance()
{
  int best = INT_MAX; Dir bestDir = UNKNOWN_DIR;
  const int prefRel[4] = {0,1,3,2};
  for (int k=0;k<4;k++){
    Dir d = (Dir)((rface + prefRel[k]) % 4);
    if (maze[rx][ry].wall[d]) continue;
    int dx,dy; dirToDelta(d, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (!inBounds(nx,ny)) continue;
    int v = distMap[nx][ny];
    if (v < best) { best = v; bestDir = d; }
  }
  return bestDir;
}

// DFS scan: explore reachable cells and populate maze[][]. Uses stack for backtracking.
static void scanMazeDFS()
{
  // init maze
  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) {
    for (int d=0; d<4; ++d) maze[x][y].wall[d] = false;
    maze[x][y].visited = false; maze[x][y].isGoal = false;
  }
  // mark outer walls
  for (int x=0;x<MAZE_N;x++){ maze[x][0].wall[SOUTH]=true; maze[x][MAZE_N-1].wall[NORTH]=true; }
  for (int y=0;y<MAZE_N;y++){ maze[0][y].wall[WEST]=true; maze[MAZE_N-1][y].wall[EAST]=true; }

  // start pose (assumed)
  rx = 0; ry = 0; rface = NORTH;
  maze[rx][ry].visited = true;

  // simple stack storing parent coordinates
  struct C { int x,y; };
  C stackArr[256]; int sp = 0;
  stackArr[sp++] = {rx,ry};

  while (sp > 0) {
    // sense current cell
    senseWallsAtCurrent();
    if (detectWhiteGoalAtCurrent()) maze[rx][ry].isGoal = true;

    // ---- DEBUG: print current cell status ----
    printStatus();


    // find unvisited neighbor with preference forward,right,left,back
    bool found = false;
    const int prefRel[4] = {0,1,3,2};
    for (int k=0;k<4 && !found;k++) {
      Dir d = (Dir)((rface + prefRel[k]) % 4);
      if (maze[rx][ry].wall[d]) continue;
      int dx,dy; dirToDelta(d,dx,dy);
      int nx=rx+dx, ny=ry+dy;
      if (!inBounds(nx,ny)) continue;
      if (!maze[nx][ny].visited) {
        // push current location as parent then move
        stackArr[sp++] = {rx,ry};
        rotateToFace(d);
        moveOneCellForward();
        maze[rx][ry].visited = true;
        found = true;
      }
    }
    if (!found) {
      // backtrack to parent
      C parent = stackArr[--sp];
      // physically move to parent cell (we assume adjacency)
      int dx = parent.x - rx;
      int dy = parent.y - ry;
      Dir desired = UNKNOWN_DIR;
      if (dx==1 && dy==0) desired = EAST;
      else if (dx==-1 && dy==0) desired = WEST;
      else if (dx==0 && dy==1) desired = NORTH;
      else if (dx==0 && dy==-1) desired = SOUTH;
      if (desired != UNKNOWN_DIR) {
        rotateToFace(desired);
        moveOneCellForward();
      } else {
        // not adjacent: break for safety
        break;
      }
    }
  } // end while
}

// Navigate to a goal using flood distances
static void navigateToGoal()
{
  computeDistancesToGoals();

  // loop until current cell is goal
  while (!maze[rx][ry].isGoal) {
    Dir next = pickBestNeighborToReduceDistance();
    if (next == UNKNOWN_DIR) { // cannot find path
      stopMotors();
      Serial.println("No path found.");
      return;
    }
    rotateToFace(next);
    moveOneCellForward();
    if (detectWhiteGoalAtCurrent()) {
      maze[rx][ry].isGoal = true;
      Serial.printf("Found goal at %d,%d\n", rx, ry);
      break;
    }
    // if we discover new walls while moving, recompute distances for safety
    senseWallsAtCurrent();
    computeDistancesToGoals();
  }
  stopMotors();
}

// Controller called from loop: chooses scan or solve based on SEARCH_MODE pin
void runFloodController()
{
  static bool scanned = false;
  int mode = digitalRead(SEARCH_MODE); // HIGH = solve(1), LOW = scan(0)
  if (mode == LOW) {
    Serial.println("Scan mode");
    scanMazeDFS();
    scanned = true;
  } else {
    if (!scanned) {
      Serial.println("Solve requested but not scanned -> quick scan.");
      scanMazeDFS();
      scanned = true;
    }
    Serial.println("Solve mode");
    navigateToGoal();
  }
}