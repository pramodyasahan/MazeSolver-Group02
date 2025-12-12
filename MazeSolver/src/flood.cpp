#include "flood.h"
#include "common.h"
#include "wall.h"
#include "line.h"
#include <Arduino.h>

// ---------------- Configuration ----------------
static int MAZE_N = 9;      // NxN logical maze
static int CELL_CM = 24;    // cell size in cm
static const int US_FRONT_TH = 12; // cm threshold to treat as wall
static const int US_SIDE_TH  = 10;

static const long INT_MAX = 1000000;

void setMazeSize(int n){ if(n>=3 && n<=15) MAZE_N = n; }
void setCellCm(int cm){ if(cm>0) CELL_CM = cm; }

// ---------------- Maze structures ----------------
struct Cell {
  bool wall[4]; // N,E,S,W
  bool visited;
  bool isGoal;
};

static Cell maze[16][16]; 
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
static bool isMazeFullyScanned()
{
  for (int x = 0; x < MAZE_N; ++x) {
    for (int y = 0; y < MAZE_N; ++y) {
      if (!maze[x][y].visited) return false;
    }
  }
  return true;
}

static bool hasWallAbsolute(int absDir)
{
  if (!inBounds(rx, ry)) return true; 
  return maze[rx][ry].wall[absDir];
}

static bool hasWallRelative(int rel)
{
  int absDir = (int)rface + rel;
  absDir %= 4;
  if (absDir < 0) absDir += 4;
  return hasWallAbsolute(absDir);
}

static const char* dirName(Dir d) {
  switch(d) {
    case NORTH: return "NORTH";
    case EAST:  return "EAST";
    case SOUTH: return "SOUTH";
    case WEST:  return "WEST";
    default:    return "UNK";
  }
}

static void printStatus()
{
  Serial.print("POS: ");
  Serial.print(rx); Serial.print(","); Serial.print(ry);
  Serial.print("  F: "); Serial.print(dirName(rface));
  Serial.print("  Walls[F/L/R]: ");

  Serial.print(hasWallRelative(0) ? "1" : "0"); Serial.print("/");
  Serial.print(hasWallRelative(-1) ? "1" : "0"); Serial.print("/");
  Serial.print(hasWallRelative(1) ? "1" : "0");

  Serial.print("  Visited: "); Serial.print(maze[rx][ry].visited ? "Y" : "N");
  Serial.print("  GoalHere: "); Serial.print(maze[rx][ry].isGoal ? "Y" : "N");

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
    smoothTurnRight(); 
  } else if (diff == -1 || diff == 3) {
    smoothTurnLeft();  
  } else if (abs(diff) == 2) {
    pivot180(PIVOT_180_PWM);
  }
  
  rface = target;
  delay(80);
}

// Move forward exactly one cell
// RETURNS TRUE ONLY IF SUCCESSFUL
static bool moveOneCellForward()
{
  // This uses the bool return we added to wall.cpp
  bool moved = moveForwardWallFollow(CELL_CM, BASE_PWM_STRAIGHT);
  stopMotors();

  if (moved) {
    // only update coordinates if physical move succeeded
    int dx,dy; dirToDelta(rface, dx, dy);
    rx += dx; ry += dy;
    return true;
  } else {
    Serial.println("ERR: Move Incomplete. Staying in cell.");
    return false;
  }
}

static bool detectWhiteGoalAtCurrent()
{
  readSensors();
  for (int i=0;i<8;i++) if (sensorVal[i]) return false;
  return true;
}

static void senseWallsAtCurrent()
{
  long front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long left  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  if (front>0 && front < US_FRONT_TH) {
    maze[rx][ry].wall[rface] = true;
    int dx,dy; dirToDelta(rface, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(rface+2)%4] = true;
  }
  
  Dir leftDir = (Dir)((rface+3)%4);
  if (left>0 && left < US_SIDE_TH) {
    maze[rx][ry].wall[leftDir] = true;
    int dx,dy; dirToDelta(leftDir, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(leftDir+2)%4] = true;
  }
  
  Dir rightDir = (Dir)((rface+1)%4);
  if (right>0 && right < US_SIDE_TH) {
    maze[rx][ry].wall[rightDir] = true;
    int dx,dy; dirToDelta(rightDir, dx, dy);
    int nx=rx+dx, ny=ry+dy;
    if (inBounds(nx,ny)) maze[nx][ny].wall[(rightDir+2)%4] = true;
  }
}

static void computeDistancesToGoals()
{
  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) distMap[x][y] = INT_MAX;

  struct P { int x,y; };
  P q[256]; int qh=0, qt=0;

  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) {
    if (maze[x][y].isGoal) {
      distMap[x][y]=0;
      q[qt++] = {x,y};
    }
  }

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

static void scanMazeDFS()
{
  // Initialization
  for (int x=0;x<MAZE_N;x++) for (int y=0;y<MAZE_N;y++) {
    for (int d=0; d<4; ++d) maze[x][y].wall[d] = false;
    maze[x][y].visited = false; maze[x][y].isGoal = false;
  }
  for (int x=0;x<MAZE_N;x++){ maze[x][0].wall[SOUTH]=true; maze[x][MAZE_N-1].wall[NORTH]=true; }
  for (int y=0;y<MAZE_N;y++){ maze[0][y].wall[WEST]=true; maze[MAZE_N-1][y].wall[EAST]=true; }

  rx = 0; ry = 0; rface = NORTH;
  maze[rx][ry].visited = true;

  struct C { int x,y; };
  C stackArr[256]; int sp = 0;
  stackArr[sp++] = {rx,ry};

  while (sp > 0) {
    senseWallsAtCurrent();
    if (detectWhiteGoalAtCurrent()) maze[rx][ry].isGoal = true;

    printStatus();

    // Look for unvisited neighbors
    bool found = false;
    const int prefRel[4] = {0,1,3,2};
    for (int k=0;k<4 && !found;k++) {
      Dir d = (Dir)((rface + prefRel[k]) % 4);
      if (maze[rx][ry].wall[d]) continue;
      
      int dx,dy; dirToDelta(d,dx,dy);
      int nx=rx+dx, ny=ry+dy;
      
      if (!inBounds(nx,ny)) continue;
      
      if (!maze[nx][ny].visited) {
        // We found an unvisited neighbor. Try to move there.
        rotateToFace(d);
        
        bool moveSuccess = moveOneCellForward();
        
        if (moveSuccess) {
          // Push *old* parent position to stack
          stackArr[sp++] = {rx - dx, ry - dy}; 
          maze[rx][ry].visited = true;
          found = true;
        } else {
          // ERROR: Failed to move (obstacle or stuck).
          // We are still at the old cell.
          // Mark this direction as a wall so we don't try it again immediately.
          maze[rx][ry].wall[d] = true;
          Serial.println("Move failed. Marking wall and retrying.");
          // found remains false, loop will continue to try next neighbor
        }
      }
    }

    if (!found) {
      // Backtracking
      C parent = stackArr[--sp];
      
      int dx = parent.x - rx;
      int dy = parent.y - ry;
      
      Dir desired = UNKNOWN_DIR;
      if (dx==1 && dy==0) desired = EAST;
      else if (dx==-1 && dy==0) desired = WEST;
      else if (dx==0 && dy==1) desired = NORTH;
      else if (dx==0 && dy==-1) desired = SOUTH;
      
      if (desired != UNKNOWN_DIR) {
        rotateToFace(desired);
        bool backSuccess = moveOneCellForward();
        if (!backSuccess) {
           Serial.println("CRITICAL: Stuck during backtracking! Halting to prevent reset.");
           // Re-push parent so we don't lose track, effectively staying in this state
           stackArr[sp++] = parent;
           stopMotors();
           while(1); // Stop here so you can see the error, instead of resetting to 0,0
        }
      } else {
        // This happens if logical position desyncs (e.g. attempting to jump diagonal)
        Serial.println("Logic Error: Non-adjacent backtrack. Halting.");
        stopMotors();
        while(1);
      }
    }
  } 
}

static void navigateToGoal()
{
  computeDistancesToGoals();
  while (!maze[rx][ry].isGoal) {
    Dir next = pickBestNeighborToReduceDistance();
    if (next == UNKNOWN_DIR) {
      stopMotors();
      Serial.println("No path found.");
      return;
    }
    rotateToFace(next);
    moveOneCellForward();
    if (detectWhiteGoalAtCurrent()) {
      maze[rx][ry].isGoal = true;
      Serial.print("Found goal at "); Serial.print(rx); Serial.print(","); Serial.println(ry);
      break;
    }
    senseWallsAtCurrent();
    computeDistancesToGoals();
  }
  stopMotors();
}

void runFloodController()
{
  static bool scanned = false;
  int mode = digitalRead(SEARCH_MODE); 
  if (mode == LOW) {
    // Only run scan if not already finished (optional check, or just run repeatedly)
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