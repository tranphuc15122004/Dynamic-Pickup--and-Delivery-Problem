import math
import random
import sys
from typing import Dict, List, Optional, Tuple
from algorithm.Object import *
from algorithm.algorithm_config import APPROACHING_DOCK_TIME, LS_MAX
from algorithm.local_search import total_cost  , dispatch_nodePair
from algorithm.local_search2 import *

class Chromosome:
    def __init__(self, vehicleid_to_plan: Dict[str, List[Node]], route_map: Dict[Tuple, Tuple], id_to_vehicle: Dict[str, Vehicle] ):
        self.solution = vehicleid_to_plan
        self.route_map = route_map
        self.id_to_vehicle = id_to_vehicle
        self.factorial_cost : List[float] = self.evaluate_fitness()
        self.fitness : float= 0
        for c in self.factorial_cost:
            self.fitness += c

    def evaluate_fitness(self) -> List[float]:
        factorial_cost = factorial_costs_of_an_individual(self.id_to_vehicle , self.route_map , self.solution)
        return  factorial_cost

    def GA_mutate(self , is_limited = False):
        # Gọi một phương pháp đột biến từ thuật toán tìm kiếm cục bộ
        GA_mutate_solution(self , is_limited)
        self.factorial_cost = self.evaluate_fitness()
        self.fitness = 0
        for c in self.factorial_cost:
            self.fitness += c
    

    def GA_crossover(self, other: 'Chromosome' , PDG_map: Dict[str , List[Node]] ) -> 'Chromosome':
        child_solution = GA_crossover_solution(self, other  ,PDG_map  )
        return child_solution
    
    def CCEA_crossover(self, other : 'Chromosome' , PDG_map : Dict[str, List[Node]] , mode = 'overtime') -> 'Chromosome':
        child_solution = CCEA_crossover_solution(self, other  ,PDG_map , mode )
        return child_solution
    
    def CCEA_mutate(self , is_limited = False , mode = 'total'):
        CCEA_mutate_solution(self  , is_limited , mode)
        self.factorial_cost = self.evaluate_fitness()
        self.fitness = 0
        for c in self.factorial_cost:
            self.fitness += c
            
    def random_mutate_operator(self , is_limited = False , mode = 'total' , is_better = 'Yes'):
        i = 0
        begin_time = time.time()
        
        while i < LS_MAX:
            is_improve = False
            selected_ls = random.choice(range(1 , 6))
            if selected_ls == 1: 
                inter_couple_exchange(self.solution , self.id_to_vehicle , self.route_map , is_limited , mode)
            elif selected_ls == 2:
                block_exchange(self.solution , self.id_to_vehicle , self.route_map , is_limited , mode)
            elif selected_ls == 3:
                block_relocate(self.solution , self.id_to_vehicle , self.route_map , is_limited , mode)
            elif selected_ls == 4:
                multi_pd_group_relocate(self.solution , self.id_to_vehicle , self.route_map , is_limited , mode)
            elif selected_ls == 5:
                improve_ci_path_by_2_opt(self.solution , self.id_to_vehicle , self.route_map , begin_time , is_limited , mode)
            
            if is_improve:
                i += 1
            else:
                i += 0.5
        
        self.factorial_cost = self.evaluate_fitness()
        self.fitness = 0
        for c in self.factorial_cost:
            self.fitness += c

    def __repr__(self):
        return f'Chromosome(Factorial fitness: {self.factorial_cost}, Fitness: {self.fitness}, Solution: {get_route_after(self.solution , {})}) '


def GA_mutate_solution(indivisual : Chromosome , is_limited = False):
    n1 , n2 , n3 , n4, n5 = 0 ,0 ,0 ,0 ,0
    begin_time = time.time()
    i  = 1
    while i < LS_MAX:
        is_improved = False
        if inter_couple_exchange(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited):
            n1 +=1
            is_improved = True
        if block_exchange(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited):
            n2 +=1
            is_improved = True
            
        if block_relocate(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited):
            is_improved = True
            n3 +=1
        if multi_pd_group_relocate(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited):
            n4 +=1
            is_improved = True
        if improve_ci_path_by_2_opt(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , begin_time , is_limited):
            n5 +=1
            is_improved = True
        if is_improved:
            i += 1
        else:
            break
    print(f"PDPairExchange:{n1}; BlockExchange:{n2}; BlockRelocate:{n3}; mPDG:{n4}; 2opt:{n5}; cost:{total_cost(indivisual.id_to_vehicle , indivisual.route_map , indivisual.solution ):.2f}" , file= sys.stderr  )

def CCEA_mutate_solution (indivisual : Chromosome, is_limited = False , mode = 'total'):
    n1 , n2 , n3 , n4, n5 = 0 ,0 ,0 ,0 ,0
    begin_time = time.time()
    i  = 1
    while i < LS_MAX:
        is_improved = False
        if inter_couple_exchange(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited , mode):
            n1 +=1
            is_improved = True
        if block_exchange(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited , mode):
            n2 +=1
            is_improved = True
        
        if block_relocate(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited , mode):
            is_improved = True
            n3 +=1
        if multi_pd_group_relocate(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , is_limited , mode):
            n4 +=1
            is_improved = True
        if improve_ci_path_by_2_opt(indivisual.solution , indivisual.id_to_vehicle , indivisual.route_map , begin_time , is_limited , mode):
            n5 +=1
            is_improved = True
        if is_improved:
            i += 1
        else:
            break
    #print(f"PDPairExchange:{n1}; BlockExchange:{n2}; BlockRelocate:{n3}; mPDG:{n4}; 2opt:{n5}; cost:{factorial_costs_of_an_individual(indivisual.id_to_vehicle , indivisual.route_map , indivisual.solution)}" , file= sys.stderr  )


def crossover_operator(indivisual1 : Chromosome , indivisual2: Chromosome , PDG_map: Dict[str , List[Node]] , mode = 'total'):
    if mode == 'overtime':
        Overtime1 = calculate_delaytime_each_vehicle(indivisual1)
        Overtime2 = calculate_delaytime_each_vehicle(indivisual2)
    elif mode == 'distance':
        Overtime1 = calculate_distance_each_vehicle(indivisual1)
        Overtime2 = calculate_distance_each_vehicle(indivisual2)
    else:
        Overtime1 = calculate_cost_each_vehicle(indivisual1)
        Overtime2 = calculate_cost_each_vehicle(indivisual2)
    Overtime = {f"{s}P1": item for s, item in Overtime1.items()}
    Overtime.update({f"{s}P2": item for s, item in Overtime2.items()})
    
    # Chuyển thành danh sách tuple
    Overtime_list = list(Overtime.items())
    random.shuffle(Overtime_list)
    Overtime = sorted(Overtime_list, key=lambda item: item[1])
    
    # Cac super node
    new_PDG_map : Dict[str , List[Node]] = {}
    for key , value in PDG_map.items():
        key = f'{len(value[0].pickup_item_list)}_{value[0].pickup_item_list[0].id}'
        new_PDG_map[key] = value
    
    # Khởi tạo lời giải con là rỗng -> điều kiện dừng của vòng lặp sẽ là kiểm tra child đã được thêm tất cả các tuyền đường từ cha và mẹ
    child_solution :Dict[str, List[Node]] = {vehicleID:[] for vehicleID in indivisual1.id_to_vehicle.keys()}
    check_valid : Dict[str , int]= {key : 0 for key in new_PDG_map.keys()}
    added_route_check : set[str] = set()
    
    # Thêm đầy đủ các tuyến đường vào lời giải con (có thể gây thừa hoặc thiếu các unongoing super node)
    while len(added_route_check) != len(indivisual1.id_to_vehicle):
        Topitem =  Overtime[0]
        del Overtime[0]
        vehicleID = Topitem[0].split('P')[0]
        ParentIndex = int(Topitem[0].split('P')[1])
        
        # Kiem tra da lay duoc dung va du tuyen duong cuar cac xe chuaw
        if vehicleID in added_route_check: 
            continue
        else:
            added_route_check.add(vehicleID)
        
        if ParentIndex == 1:
            for node in indivisual1.solution[vehicleID]:
                child_solution[vehicleID].append(node)
        else:
            for node in indivisual2.solution[vehicleID]:
                child_solution[vehicleID].append(node)

        # Lưu các nút thừa trong tuyến đường hiện tại
        redundant = []
        del_index = []
        # Duyệt ngược danh sách để tìm và xóa nút thừa    
        for i in range(len(child_solution[vehicleID]) - 1, -1, -1):  
            node = child_solution[vehicleID][i]
            
            if node.pickup_item_list:
                if redundant and node.pickup_item_list[0].id == redundant[-1]:
                    redundant.pop()  # Loại bỏ phần tử tương ứng trong danh sách `redundant`
                    del_index.append(i)
            else:
                key = f'{len(node.delivery_item_list)}_{node.delivery_item_list[-1].id}'
                
                if key in new_PDG_map:
                    check_valid[key] += 1
                    
                    # nếu tìm được một super node thừa
                    if check_valid[key] > 1:
                        first_itemID_of_redundant_supernode = key.split('_')[-1]
                        redundant.append(first_itemID_of_redundant_supernode)
                        #print(f"Redundant nodes: {redundant}" , file= sys.stderr)

                        # Xóa node giao của super node thừa
                        del_index.append(i)
                        #print('Đã xóa 1 super node thừa' , file= sys.stderr)
        for i in del_index:
            child_solution[vehicleID].pop(i)

    # Kiem tra lai solution        
    for key, value in check_valid.items():
        if value == 0:
            # truong hop bi thieu 1 super node thi gan theo chien luoc CI vao solution hien tai
            selected_vehicleID = random.choice(list(indivisual1.id_to_vehicle.keys()))
            node_list = new_PDG_map[key]
            isExhausive = False
            route_node_list : List[Node] = []
            
            if node_list:
                isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(node_list , indivisual1.id_to_vehicle , child_solution , indivisual1.route_map ,selected_vehicleID , mode)
                
            route_node_list = child_solution.get(bestInsertVehicleID , [])

            if isExhausive:
                route_node_list = bestNodeList[:]
            else:
                if route_node_list is None:
                    route_node_list = []
                
                new_order_pickup_node = node_list[0]
                new_order_delivery_node = node_list[1]
                
                route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                route_node_list.insert(bestInsertPosJ, new_order_delivery_node)
            child_solution[bestInsertVehicleID] = route_node_list
            
            #print('Cập nhật super node còn thiếu' , file= sys.stderr)
    
    sorted_child_solution = sorted(child_solution.items() ,  key=lambda x: int(x[0].split('_')[1]))
    child_solution.clear()
    child_solution.update(sorted_child_solution)
    child = Chromosome(child_solution , indivisual1.route_map , indivisual1.id_to_vehicle)
    return child

def GA_crossover_solution(indivisual1 : Chromosome , indivisual2: Chromosome , PDG_map: Dict[str , List[Node]] ):
    return crossover_operator(indivisual1 , indivisual2 , PDG_map , 'overtime')

def CCEA_crossover_solution(indivisual1 : Chromosome , indivisual2: Chromosome , PDG_map: Dict[str , List[Node]] , mode = 'overtime'):
    return crossover_operator(indivisual1 , indivisual2 , PDG_map , mode)

def calculate_delaytime_each_vehicle(chromosome: Chromosome) -> Dict[str , int]:
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(chromosome.id_to_vehicle)
    # asdfasdfasdfasdfasdfasd
    overtime_per_vehicle : Dict[str , int] = {}
    for vehicleID in chromosome.id_to_vehicle.keys():
        overtime_per_vehicle[vehicleID] = 0

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in chromosome.id_to_vehicle.items():
        time  = 0
        
        if otherVehicle.cur_factory_id :
            if otherVehicle.leave_time_at_current_factory > otherVehicle.gps_update_time:
                tw: List[int] = [
                    otherVehicle.arrive_time_at_current_factory,
                    otherVehicle.leave_time_at_current_factory
                ]
                tw_list: Optional[List[List[int]]] = dock_table.get(otherVehicle.cur_factory_id)
                if tw_list is None:
                    tw_list = []
                tw_list.append(tw)
                dock_table[otherVehicle.cur_factory_id] = tw_list
            leave_last_node_time[index] = otherVehicle.leave_time_at_current_factory
        else:
            leave_last_node_time[index] = otherVehicle.gps_update_time
        
        if chromosome.solution.get(vehicleID) and len(chromosome.solution.get(vehicleID)) > 0:    
            curr_node[index] = 0
            n_node[index] = len(chromosome.solution[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == chromosome.solution[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = chromosome.route_map.get((otherVehicle.cur_factory_id , chromosome.solution[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == chromosome.solution[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = chromosome.route_map.get((otherVehicle.cur_factory_id , chromosome.solution[vehicleID][0].id))
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
        
    while n > 0:
        minT = math.inf
        minT2VehicleIndex = 0
        tTrue = minT
        idx = 0
        
        for i in range (vehicle_num):
            if curr_time[i] < minT:
                minT = curr_time[i]
                minT2VehicleIndex = i
        
        minT2VehicleIndex += 1
        minT2VehicleID = "V_" + str(minT2VehicleIndex)
        minT2VehicleIndex -= 1
        
        minTNodeList: List[Node] = []
        minTNodeList = chromosome.solution.get(minT2VehicleID)
        minTNode = minTNodeList[curr_node[minT2VehicleIndex]]
        
        if minTNode.delivery_item_list and len(minTNode.delivery_item_list) > 0:
            beforeOrderID = ""
            nextOrderID = ""
            for order_item in minTNode.delivery_item_list:
                nextOrderID = order_item.id
                if beforeOrderID != nextOrderID:
                    commitCompleteTime = order_item.committed_completion_time
                    overtime_per_vehicle[minT2VehicleID] += max(0 , curr_time[minT2VehicleIndex] - commitCompleteTime)
                beforeOrderID = nextOrderID
        
        usedEndTime : List[int] = []
        timeSlots : List[List[int]] =  dock_table.get(minTNode.id, [])
        if timeSlots:
            i = 0
            while i < len(timeSlots):
                time_slot = timeSlots[i]
                if time_slot[1] <= minT:
                    timeSlots.pop(i)  # Xóa phần tử nếu end_time <= minT
                elif time_slot[0] <= minT < time_slot[1]:
                    usedEndTime.append(time_slot[1])
                    i += 1
                else:
                    print("------------ timeslot.start > minT --------------", file = sys.stderr)
                    i += 1

        if len(usedEndTime) < 6:
            tTrue = minT
        else:
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]
            
        service_time = minTNodeList[curr_node[minT2VehicleIndex]].service_time
        cur_factory_id = minTNodeList[curr_node[minT2VehicleIndex]].id
        curr_node[minT2VehicleIndex] += 1

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and
            cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].id):

            delivery_item_list = minTNodeList[curr_node[minT2VehicleIndex]].delivery_item_list
            
            if delivery_item_list and len(delivery_item_list) > 0:
                before_order_id = ""
                next_order_id = ""

                for order_item in delivery_item_list:
                    next_order_id = order_item.order_id
                    if before_order_id != next_order_id:
                        commit_complete_time = order_item.committed_completion_time
                        overtime_per_vehicle[minT2VehicleID] += max(0, curr_time[minT2VehicleIndex] - commit_complete_time)
                    before_order_id = next_order_id

            service_time += minTNodeList[curr_node[minT2VehicleIndex]].service_time
            curr_node[minT2VehicleIndex] += 1
            
        if curr_node[minT2VehicleIndex] >= n_node[minT2VehicleIndex]:
            n -= 1
            curr_node[minT2VehicleIndex] = math.inf
            curr_time[minT2VehicleIndex] = math.inf
            n_node[minT2VehicleIndex] = 0
        else:
            dis_and_time = chromosome.route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                time = int(dis_and_time[1])

                curr_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    return overtime_per_vehicle

def calculate_distance_each_vehicle (chromosome: Chromosome) -> Dict[str , float]:
    overtime_Sum : float = 0.0
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(chromosome.id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    # khoang cach di chuyen cua tung xe
    distance_per_vehcle : Dict[str , float]= {}
    for vehicleID in chromosome.id_to_vehicle.keys():
        distance_per_vehcle[vehicleID] = 0
    
    for vehicleID , otherVehicle in chromosome.id_to_vehicle.items():
        distance = 0
        time  = 0
        
        if otherVehicle.cur_factory_id :
            if otherVehicle.leave_time_at_current_factory > otherVehicle.gps_update_time:
                tw: List[int] = [
                    otherVehicle.arrive_time_at_current_factory,
                    otherVehicle.leave_time_at_current_factory
                ]
                tw_list: Optional[List[List[int]]] = dock_table.get(otherVehicle.cur_factory_id)
                if tw_list is None:
                    tw_list = []
                tw_list.append(tw)
                dock_table[otherVehicle.cur_factory_id] = tw_list
            leave_last_node_time[index] = otherVehicle.leave_time_at_current_factory
        else:
            leave_last_node_time[index] = otherVehicle.gps_update_time
        
        if chromosome.solution.get(vehicleID) and len(chromosome.solution.get(vehicleID)) > 0:    
            curr_node[index] = 0
            n_node[index] = len(chromosome.solution[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == chromosome.solution[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = chromosome.route_map.get((otherVehicle.cur_factory_id , chromosome.solution[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    distance_per_vehcle[vehicleID] += distance
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == chromosome.solution[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = chromosome.route_map.get((otherVehicle.cur_factory_id , chromosome.solution[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        distance_per_vehcle[vehicleID] += distance
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
        
    while n > 0:
        minT = math.inf
        minT2VehicleIndex = 0
        tTrue = minT
        idx = 0
        
        for i in range (vehicle_num):
            if curr_time[i] < minT:
                minT = curr_time[i]
                minT2VehicleIndex = i
        
        minT2VehicleIndex += 1
        minT2VehicleID = "V_" + str(minT2VehicleIndex)
        minT2VehicleIndex -= 1
        
        minTNodeList: List[Node] = []
        minTNodeList = chromosome.solution.get(minT2VehicleID)
        minTNode = minTNodeList[curr_node[minT2VehicleIndex]]
        
        if minTNode.delivery_item_list and len(minTNode.delivery_item_list) > 0:
            beforeOrderID = ""
            nextOrderID = ""
            for order_item in minTNode.delivery_item_list:
                nextOrderID = order_item.id
                if beforeOrderID != nextOrderID:
                    commitCompleteTime = order_item.committed_completion_time
                    overtime_Sum += max(0 , curr_time[minT2VehicleIndex] - commitCompleteTime)
                beforeOrderID = nextOrderID
        
        usedEndTime : List[int] = []
        timeSlots : List[List[int]] =  dock_table.get(minTNode.id, [])
        if timeSlots:
            i = 0
            while i < len(timeSlots):
                time_slot = timeSlots[i]
                if time_slot[1] <= minT:
                    timeSlots.pop(i)  # Xóa phần tử nếu end_time <= minT
                elif time_slot[0] <= minT < time_slot[1]:
                    usedEndTime.append(time_slot[1])
                    i += 1
                else:
                    print("------------ timeslot.start > minT --------------", file = sys.stderr)
                    i += 1

        if len(usedEndTime) < 6:
            tTrue = minT
        else:
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]
            
        service_time = minTNodeList[curr_node[minT2VehicleIndex]].service_time
        cur_factory_id = minTNodeList[curr_node[minT2VehicleIndex]].id
        curr_node[minT2VehicleIndex] += 1

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and
            cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].id):

            delivery_item_list = minTNodeList[curr_node[minT2VehicleIndex]].delivery_item_list
            
            if delivery_item_list and len(delivery_item_list) > 0:
                before_order_id = ""
                next_order_id = ""

                for order_item in delivery_item_list:
                    next_order_id = order_item.order_id
                    if before_order_id != next_order_id:
                        commit_complete_time = order_item.committed_completion_time
                        overtime_Sum += max(0, curr_time[minT2VehicleIndex] - commit_complete_time)
                    before_order_id = next_order_id

            service_time += minTNodeList[curr_node[minT2VehicleIndex]].service_time
            curr_node[minT2VehicleIndex] += 1
            
        if curr_node[minT2VehicleIndex] >= n_node[minT2VehicleIndex]:
            n -= 1
            curr_node[minT2VehicleIndex] = math.inf
            curr_time[minT2VehicleIndex] = math.inf
            n_node[minT2VehicleIndex] = 0
        else:
            dis_and_time = chromosome.route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                distance_per_vehcle[minT2VehicleID] += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    return distance_per_vehcle

def calculate_cost_each_vehicle (chromosome: Chromosome)-> Dict[str , float]:
    cost_per_vehcle : Dict[str , float]= {}
    for vehicleID in chromosome.id_to_vehicle.keys():
        cost_per_vehcle[vehicleID] = 0
    
    for vehicleID , plan in chromosome.solution.items():
        vehicle =  chromosome.id_to_vehicle[vehicleID]
        cost_per_vehcle[vehicleID] = single_vehicle_cost(plan , vehicle , chromosome.route_map)
    
    return cost_per_vehcle