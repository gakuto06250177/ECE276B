import numpy as np

def intersects_aabb(p1, p2, block):
  
  d = p2 - p1

  x_min,y_min,z_min,x_max,y_max,z_max = block[:6]

  if d[0] == 0 and (p1[0] < x_min or p1[0] > x_max):
    return False
  if d[1] == 0 and (p1[1] < y_min or p1[1] > y_max):
    return False
  if d[2] == 0 and (p1[2] < z_min or p1[2] > z_max):
    return False

  tx,ty,tz = [-np.inf, np.inf], [-np.inf, np.inf], [-np.inf, np.inf]
  
  if d[0] != 0:
    tx = sorted([(x_min - p1[0]) / d[0], (x_max - p1[0]) / d[0]])

  if d[1] != 0:
    ty = sorted([(y_min - p1[1]) / d[1], (y_max - p1[1]) / d[1]])

  if d[2] != 0:
    tz = sorted([(z_min - p1[2]) / d[2], (z_max - p1[2]) / d[2]])

  t_min = max(tx[0], ty[0], tz[0])
  t_max = min(tx[1], ty[1], tz[1])

  return True if t_min <= t_max and t_max >= 0 and t_min <= 1 else False

class MyPlanner:
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def is_inside_boundary(self, point):
    return (self.boundary[0,0] <= point[0] <= self.boundary[0,3] and
            self.boundary[0,1] <= point[1] <= self.boundary[0,4] and
            self.boundary[0,2] <= point[2] <= self.boundary[0,5])

  def is_point_in_collision(self, point):
    for k in range(self.blocks.shape[0]):
      if (self.blocks[k,0] <= point[0] <= self.blocks[k,3] and
          self.blocks[k,1] <= point[1] <= self.blocks[k,4] and
          self.blocks[k,2] <= point[2] <= self.blocks[k,5]):
        return True
    return False

  def is_valid_point(self, point):
    return self.is_inside_boundary(point) and not self.is_point_in_collision(point)

  def plan(self,start,goal):
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]

        if not self.is_valid_point(next):
          continue
        
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

class MyPart1SamplingPlanner(MyPlanner):
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def plan(self, start, goal):
    path = [start]
    numofdirs = 26
    
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        start_point = path[-1]
        next_vertical = dR[:,k]

        for t in np.linspace(0,1,10):
          next = start_point + t*next_vertical

          if not self.is_valid_point(next):
            continue
          
          disttogoal = sum((next - goal)**2)
          if( disttogoal < mindisttogoal):
            mindisttogoal = disttogoal
            node = next
        
      if node is None:
        break
      
      path.append(node)
      
      if sum((path[-1]-goal)**2) <= 0.1:
        break
    
    return np.array(path)
  
class MyPart1AABBPlanner(MyPlanner):
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def plan(self,start,goal):
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        for block in self.blocks:

          if intersects_aabb(path[-1], next, block):
            next = path[-1] + dR[:,k]

          if not self.is_valid_point(next):
            continue
          
          disttogoal = sum((next - goal)**2)
          if( disttogoal < mindisttogoal):
            mindisttogoal = disttogoal
            node = next
      
      if node is None:
        break
      
      path.append(node)
      
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)
  
  
# Part 2: A* Planner
class MyAStarPlanner(MyPlanner):
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks
    # ここに、衝突判定やヒューリスティック計算に必要な情報を
    # AStarクラスに渡せるように準備しておくと良い
  
  def plan(self, start, goal):
    # astar.py に実装したA*のロジックを呼び出す
    # 'self' を environmentとして渡すことで、AStarクラス内で
    # 衝突判定などのメソッドが使えるようになる
    path_coords = AStar.plan(start, goal, self) 
    
    return np.array(path_coords)

  # AStarクラスから呼び出されるヘルパー関数をここに実装
  def getHeuristic(self, coord):
      # ゴールまでのユークリッド距離などを計算
      pass
  
  def getNeighbors(self, coord):
      # 現在地から移動可能な近隣ノードのリストを返す
      pass

  def checkCollision(self, p1, p2):
      # Part 1 で実装した衝突判定関数
      pass

# Part 3: RRT Planner (将来的に追加)
class MyRRTPlanner:
  __slots__ = ['boundary', 'blocks']

  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks
    # RRTに必要な情報をここで初期化

  def plan(self, start, goal):
    # RRTアルゴリズムをここに実装
    pass
