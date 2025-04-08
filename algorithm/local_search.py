import json
import os
import sys
import math
from typing import Dict , List, Optional , Tuple 
from algorithm.Object import *
import copy
from algorithm.algorithm_config import APPROACHING_DOCK_TIME , Delta1, Delta , debugPeriod , SLACK_TIME_THRESHOLD , modle4 , modle6 , modle8
from datetime import datetime


def create_Pickup_Delivery_nodes(tmp_itemList: list[OrderItem] , id_to_factory: Dict[str , Factory]) -> list[Node]:
    res: list[Node] = []
    if tmp_itemList:
        pickup_address =tmp_itemList[0].pickup_factory_id
        delivery_address =  tmp_itemList[0].delivery_factory_id
        for order_item in tmp_itemList:
            if order_item.pickup_factory_id != pickup_address:
                print("The pickup factory of these items is not the same" , file = sys.stderr)
                pickup_address = ""
                break

        for order_item in tmp_itemList:
            if order_item.delivery_factory_id != delivery_address:
                print("The delivery factory of these items is not the same" , file = sys.stderr)
                delivery_address = ""
                break
    else:
        return []

    if len(pickup_address) ==0 or len(delivery_address) == 0:
        return []
    
    pickup_factory = id_to_factory[pickup_address]
    delivery_factory = id_to_factory[delivery_address]

    pickup_item_list = []
    for item in tmp_itemList:
        pickup_item_list.append(item)
    pickup_node = Node(factory_id= pickup_factory.factory_id , delivery_item_list=[] , pickup_item_list= pickup_item_list , lng= pickup_factory.lng , lat= pickup_factory.lat)

    delivery_item_list = []
    for item in reversed(tmp_itemList):
        delivery_item_list.append(item)
    delivery_node = Node(delivery_factory.factory_id,delivery_item_list,[],delivery_factory.lng,delivery_factory.lat)

    res.extend([pickup_node, delivery_node])
    return res

def dispatch_nodePair(node_list: list[Node]  , id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str, list[Node]], route_map: Dict[tuple , tuple]  , selected_vehicle: str= None , mode = 'total' ):
    bestInsertVehicleID: str = ''
    bestInsertPosI: int = 0
    bestInsertPosJ: int = 1
    bestNodeList : list[Node] = []
    isExhausive  = False
    new_pickup_node = node_list[0]
    new_delivery_node = node_list[1]
    minCostDelta = math.inf

    for vehicleID , vehicle in id_to_vehicle.items():
        if selected_vehicle is not None and vehicleID != selected_vehicle:
            continue
        vehicle_plan = vehicleid_to_plan[vehicleID]
        
        node_list_size = len(vehicle_plan) if vehicle_plan else 0

        insert_pos = 0 
        model_nodes_num = node_list_size + 2
        first_merge_node_num = 0

        if vehicle.des:
            if new_pickup_node.id != vehicle.des.id:
                insert_pos = 1
            
            if vehicle_plan is not None and vehicle_plan:
                for node in vehicle_plan:
                    if vehicle.des.id != node.id:
                        break
                    first_merge_node_num += 1

        model_nodes_num -= first_merge_node_num

        modle_node_list : List[Node]= [] # thêm các cặp node gửi và nhận theo thứ tự mới (tuần tự)
        exhaustive_route_node_list : List[Node]= [] # Dùng để lưu giữ kế hoạch của một xe trong quá trình duyệt tham lam
        cp_route_node_list : List[Node] = [] # Một copy của một kế hoạch hiện có 
        if vehicle_plan:
            for node in vehicle_plan:
                cp_route_node_list.append(node)

        empty_pos_num = 0
        
        if model_nodes_num <= 8:
            if first_merge_node_num > 0:
                while first_merge_node_num > 0:
                    exhaustive_route_node_list.append(cp_route_node_list.pop(0))
                    first_merge_node_num -= 1
            
            count = 0
            i = 0
            while True:
                if (not cp_route_node_list) or (len(cp_route_node_list) == 0) or (i >= len(cp_route_node_list)):
                    break
                pickup_node = None
                delivery_node = None
                order_item_id = ""
                
                # Kiểm tra pickup_item_list
                if (cp_route_node_list[i].pickup_item_list  and len(cp_route_node_list[i].pickup_item_list) > 0):
                    order_item_id = cp_route_node_list[i].pickup_item_list[0].id
                    pickup_node = cp_route_node_list[i]
                    del cp_route_node_list[i]  # Xóa phần tử tại i
                    
                    # Tìm delivery node
                    j = i
                    while j < len(cp_route_node_list):
                        if (cp_route_node_list[j].delivery_item_list is not None and len(cp_route_node_list[j].delivery_item_list) > 0):
                            item_id = cp_route_node_list[j].delivery_item_list[- 1].id
                            if order_item_id == item_id:  
                                delivery_node = cp_route_node_list[j]
                                del cp_route_node_list[j] 
                                break
                        j += 1
                    
                    # Thêm pickup_node và delivery_node vào modle_node_list tại vị trí count
                    modle_node_list.insert(count, pickup_node)
                    modle_node_list.insert(count + 1, delivery_node)
                    i -= 1  # Giảm i vì danh sách đã bị xóa phần tử
                    count += 1
                
                i += 1

            # Thêm new_order_pickup_node và new_order_delivery_node
            modle_node_list.insert(count, new_pickup_node)
            count += 1
            modle_node_list.insert(count, new_delivery_node)

            empty_pos_num = len(cp_route_node_list) if cp_route_node_list else 0 
            
            while cp_route_node_list:
                modle_node_list.append(cp_route_node_list.pop(0))

            model_nodes_num = len(modle_node_list) + empty_pos_num

        if model_nodes_num <= 8:
            if model_nodes_num == 2:
                exhaustive_route_node_list.append(new_pickup_node)
                exhaustive_route_node_list.append(new_delivery_node)
                tmp_cost = cost_of_a_route(exhaustive_route_node_list , vehicle , id_to_vehicle , route_map , vehicleid_to_plan , mode)

                if tmp_cost < minCostDelta: 
                    minCostDelta = tmp_cost
                    isExhausive = True
                    bestInsertVehicleID = vehicleID
                    bestNodeList = exhaustive_route_node_list[:]
            elif model_nodes_num == 4:
                for i in range(len(modle4)):
                    if empty_pos_num == 1:
                        if modle4[i][0] == 0:
                            for j in range(1, 4):
                                exhaustive_route_node_list.append(modle_node_list[modle4[i][j] - 1])

                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan , mode)

                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-3:]
                    else:
                        for j in range(4):
                            exhaustive_route_node_list.append(modle_node_list[modle4[i][j]])

                        costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan, mode)

                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True
                        del exhaustive_route_node_list[-4:]
            elif model_nodes_num == 6:
                for i in range(len(modle6)):
                    if empty_pos_num == 1:
                        if modle6[i][0] == 0:
                            for j in range(1, 6):
                                exhaustive_route_node_list.append(modle_node_list[modle6[i][j] - 1])
                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan, mode)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-5:]
                    elif empty_pos_num == 2:
                        if modle6[i][0] == 0 and modle6[i][1] == 1:
                            for j in range(2, 6):
                                exhaustive_route_node_list.append(modle_node_list[modle6[i][j] - 2])
                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle ,id_to_vehicle ,route_map , vehicleid_to_plan , mode)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-4:]
                    else:
                        for j in range(6):
                            exhaustive_route_node_list.append(modle_node_list[modle6[i][j]])
                        costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan, mode)
                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True
                        del exhaustive_route_node_list[-6:]
            elif model_nodes_num == 8:
                for i in range(len(modle8)):
                    if empty_pos_num == 1:
                        if modle8[i][0] == 0:
                            for j in range(1, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 1])
                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan, mode)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-7:]
                    elif empty_pos_num == 2:
                        if modle8[i][0] == 0 and modle8[i][1] == 1:
                            for j in range(2, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 2])
                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan, mode)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-6:]
                    elif empty_pos_num == 3:
                        if modle8[i][0] == 0 and modle8[i][1] == 1 and modle8[i][2] == 2:
                            for j in range(3, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 3])
                            costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan, mode)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-5:]
                    else:
                        for j in range(8):
                            exhaustive_route_node_list.append(modle_node_list[modle8[i][j]])
                        costValue = cost_of_a_route(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan, mode)
                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True
                        del exhaustive_route_node_list[-8:]
            
        else:
            
            for i in range(insert_pos, node_list_size + 1):
                if vehicle_plan is not None:
                    tempRouteNodeList = copy.deepcopy(vehicle_plan)
                else:
                    tempRouteNodeList = []

                tempRouteNodeList.insert(i, new_pickup_node)

                for j in range(i + 1, node_list_size + 2):
                    if j != i + 1 and tempRouteNodeList[j - 1].pickup_item_list:
                        for k in range(j, node_list_size + 2):
                            if tempRouteNodeList[k].delivery_item_list:
                                if tempRouteNodeList[j - 1].pickup_item_list[0].id == tempRouteNodeList[k].delivery_item_list[- 1].id:
                                    j = k + 1
                                    break

                    elif tempRouteNodeList[j - 1].delivery_item_list :
                        is_terminal = True
                        for k in range(j - 2, -1, -1):
                            if tempRouteNodeList[k].pickup_item_list:
                                if tempRouteNodeList[j - 1].delivery_item_list[- 1].id == tempRouteNodeList[k].pickup_item_list[0].id:
                                    if k < i:
                                        is_terminal = True
                                        break
                                    elif k > i:
                                        is_terminal = False
                                        break
                        if is_terminal:
                            break

                    tempRouteNodeList.insert(j, new_delivery_node)

                    costValue = cost_of_a_route(tempRouteNodeList, vehicle, id_to_vehicle , route_map , vehicleid_to_plan, mode)
                    if costValue < minCostDelta:
                        minCostDelta = costValue
                        bestInsertPosI = i
                        bestInsertPosJ = j
                        bestInsertVehicleID = vehicleID
                        isExhausive = False

                    tempRouteNodeList.pop(j)
        #print(minCostDelta , , file = sys.stderr)
    #print(f"Best cost {minCostDelta}" , , file = sys.stderr)
    return isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList


# Truyền tham số bình thường vào hàm này (Không cần truyền Copy)
def get_UnongoingSuperNode (vehicleid_to_plan: Dict[str , List[Node]] , id_to_vehicle: Dict[str , Vehicle] ) -> Dict[int , Dict[str, Node]]:
    UnongoingSuperNodes : Dict[int , Dict[str , Node]] = {}
    #print()
    NodePairNum = 0 
    # xet từng kế hoạch di chuyển của phương tiện
    for vehicleID , vehicle_plan in vehicleid_to_plan.items():
        vehicle = id_to_vehicle[vehicleID]
        #print(len(vehicle_plan) , file = sys.stderr)
        if vehicle_plan and len(vehicle_plan) > 0:
            index = 1 if vehicle.des else 0
            
            # Để duy trì tính FILO 
            pickup_node_heap : List[Node] = []
            p_node_idx_heap : List[int] = []
            p_and_d_node_map : Dict[str , Node] = {}
            before_p_factory_id , before_d_factory_id = None , None
            before_p_node_idx , before_d_node_idx = 0 , 0
            
            # đối với mỗi node của phương tiện
            for i in range (index , len(vehicle_plan)):
                curr = vehicle_plan[i]
                if curr.delivery_item_list  and curr.pickup_item_list :
                    print ("Exits combine node exception when Local search" , file= sys.stderr)
                
                heapTopOrderItemId = pickup_node_heap[0].pickup_item_list[0].id if pickup_node_heap else ""
                # Nếu node hiện tại là node giao
                if curr.delivery_item_list:
                    # Nó là node giao của node nhận đầu tiên trong heap
                    if curr.delivery_item_list[-1].id == heapTopOrderItemId:
                        pickup_node_key = f"{vehicleID},{p_node_idx_heap[0]}"
                        delivery_node_key = f"{vehicleID},{int(i)}"
                
                        if len(p_and_d_node_map) >= 2:
                            if ((pickup_node_heap[0].id != before_p_factory_id) or (p_node_idx_heap[0] + 1 != before_p_node_idx) or (curr.id != before_d_factory_id) or (i - 1 != before_d_node_idx)):

                                while p_and_d_node_map:
                                    temp = dict(list(p_and_d_node_map.items())[:2])
                                    UnongoingSuperNodes[NodePairNum] = temp
                                    #print(UnongoingSuperNodes, file = sys.stderr)
                                    NodePairNum += 1
                                    for key in temp.keys():
                                        del p_and_d_node_map[key]

                        p_and_d_node_map[pickup_node_key] = pickup_node_heap[0]
                        p_and_d_node_map[delivery_node_key] = curr

                        before_p_factory_id = pickup_node_heap[0].id
                        before_p_node_idx = p_node_idx_heap[0]
                        before_d_factory_id = curr.id
                        before_d_node_idx = i
                        pickup_node_heap.pop(0)
                        p_node_idx_heap.pop(0)
                
                # Nếu node hiện tại là node nhận
                if curr.pickup_item_list:
                    pickup_node_heap.insert(0 , curr)
                    p_node_idx_heap.insert(0 , i)
                    if p_and_d_node_map:
                        while p_and_d_node_map:
                            temp = dict(list(p_and_d_node_map.items())[:2])
                            UnongoingSuperNodes[NodePairNum] = temp
                            #print(UnongoingSuperNodes, file = sys.stderr)
                            NodePairNum += 1
                            for key in temp.keys():
                                del p_and_d_node_map[key]
            
            if len(p_and_d_node_map) >= 2:
                
                # Đã sửa phần này
                while p_and_d_node_map:
                    temp = dict(list(p_and_d_node_map.items())[:2])
                    UnongoingSuperNodes[NodePairNum] = temp
                    #print(UnongoingSuperNodes, file = sys.stderr)
                    NodePairNum += 1
                    for key in temp.keys():
                        del p_and_d_node_map[key]
                p_and_d_node_map = {}
                
    return UnongoingSuperNodes


def isFeasible(route_node_list : List[Node] , carrying_items : List[OrderItem] , capacity : float ):
    unload_item_list = carrying_items[::-1] if carrying_items else []

    for node in route_node_list:
        delivery_items : List[OrderItem] = node.delivery_item_list
        pickup_items : List[OrderItem]= node.pickup_item_list

        if delivery_items:
            for order_item in delivery_items:
                if (not unload_item_list) or (unload_item_list[0] is None) or (unload_item_list[0].id != order_item.id):
                    #print("Violate FILO 1" , file= sys.stderr)
                    return False
                del unload_item_list[0]

        if pickup_items:
            for orderitem in pickup_items:
                unload_item_list.insert(0 , orderitem)
    
    if unload_item_list:
        #print("Violate FILO 2" ,file= sys.stderr)
        return False

    left_capacity = capacity
    if carrying_items:
        for order_item in carrying_items:
            left_capacity -= order_item.demand

    for node in route_node_list:
        delivery_items = node.delivery_item_list
        pickup_items = node.pickup_item_list

        if delivery_items:
            for order_item in delivery_items:
                left_capacity += order_item.demand
                if left_capacity > capacity:
                    return False

        if pickup_items:
            for order_item in pickup_items:
                left_capacity -= order_item.demand
                if left_capacity < 0:
                    return False

    return (not unload_item_list)


def cost_of_a_route (temp_route_node_list : List[Node] , vehicle: Vehicle , id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]] , mode = 'total') -> float:
    curr_factoryID = vehicle.cur_factory_id
    driving_dis  : float = 0.0
    overtime_Sum : float = 0.0
    objF : float = 0.0
    capacity = vehicle.board_capacity
    carrying_Items : List[OrderItem] = vehicle.carrying_items if vehicle.des else []
    
    if (temp_route_node_list) and (not isFeasible(temp_route_node_list , carrying_Items , capacity)):
        return math.inf
    
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in id_to_vehicle.items():
        distance = 0
        time  = 0
        
        if otherVehicle.cur_factory_id:
            if otherVehicle.leave_time_at_current_factory > otherVehicle.gps_update_time:
                tw: List[int] = [
                    otherVehicle.arrive_time_at_current_factory,
                    otherVehicle.leave_time_at_current_factory
                ]
                tw_list: Optional[List[List[int]]] = dock_table.get(otherVehicle.cur_factory_id , [])
                tw_list.append(tw)
                dock_table[otherVehicle.cur_factory_id] = tw_list
            leave_last_node_time[index] = otherVehicle.leave_time_at_current_factory
        else:
            leave_last_node_time[index] = otherVehicle.gps_update_time
        
        # Neu la xe dang xet
        if vehicleID == vehicle.id:
            if not temp_route_node_list or len(temp_route_node_list) == 0:
                curr_node[index] = math.inf
                curr_time[index] = math.inf
                n_node[index] = 0
            else:
                curr_node[index] = 0
                n_node[index] = len(temp_route_node_list)
                if not vehicle.des:
                    if vehicle.cur_factory_id == "":
                        print("cur factory have no value" , file= sys.stderr)
                    if len(temp_route_node_list) == 0:
                        print("tempRouteNodeList have no length" , file= sys.stderr)
                        
                    if vehicle.cur_factory_id == temp_route_node_list[0].id:
                        curr_time[index] = vehicle.leave_time_at_current_factory
                    else:
                        dis_and_time = route_map.get((vehicle.cur_factory_id , temp_route_node_list[0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] = vehicle.leave_time_at_current_factory + time
                        driving_dis += distance
                else:
                    if curr_factoryID:
                        if  curr_factoryID != temp_route_node_list[0].id:
                            curr_time[index] = vehicle.leave_time_at_current_factory
                            dis_and_time = route_map.get((curr_factoryID , temp_route_node_list[0].id))
                            distance = float(dis_and_time[0])
                            time = int(dis_and_time[1])
                            driving_dis += distance
                            curr_time[index] += time
                        else:
                            curr_time[index] = vehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = vehicle.des.arrive_time
                n += 1
        # Neu khong phai xe dang xet va co tuyen duong
        elif vehicleid_to_plan[vehicleID] and len(vehicleid_to_plan[vehicleID]) > 0:    
            curr_node[index] = 0
            n_node[index] = len(vehicleid_to_plan[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    driving_dis += distance
            else:
                if otherVehicle.cur_factory_id and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        driving_dis += distance
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
    
    while (n > 0):
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
        if minT2VehicleID == vehicle.id:
            minTNodeList = temp_route_node_list
        else:
            minTNodeList = vehicleid_to_plan[minT2VehicleID]
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

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].id):

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
            dis_and_time = route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                driving_dis += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    objF = (Delta * overtime_Sum) + (driving_dis / float(len(id_to_vehicle)))
    if objF < 0:
        print("the objective function less than 0" , file= sys.stderr)
        
    if mode == 'overtime':
        return (Delta * overtime_Sum)
    elif mode  == 'distance':
        return (driving_dis / float(len(id_to_vehicle)))
    return objF


def total_cost(id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]]) -> float:
    driving_dis  : float = 0.0
    overtime_Sum : float = 0.0
    objF : float = 0.0
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in id_to_vehicle.items():
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
        
        if vehicleid_to_plan.get(vehicleID) and len(vehicleid_to_plan.get(vehicleID)) > 0:    
            curr_node[index] = 0
            n_node[index] = len(vehicleid_to_plan[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    driving_dis += distance
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        driving_dis += distance
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
        minTNodeList = vehicleid_to_plan.get(minT2VehicleID)
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

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].id):

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
            dis_and_time = route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                driving_dis += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    objF = (Delta * overtime_Sum) + (driving_dis / float(len(id_to_vehicle)))
    if objF < 0:
        print("the objective function less than 0" , file= sys.stderr)
    return objF

# tinh chi phi di chuyen cua mot xe
def single_vehicle_cost(route_node_list: List[Node] , vehicle:Vehicle , route_map: Dict[tuple[str , str] , tuple[str , str]] , mode = 'total'):
    curr_factoryID = vehicle.cur_factory_id
    utilTime :int = 0
    driving_dis :float= 0.0
    overtime_sum :float= 0.0
    orderNum :int= 0
    objF: float  = 0.0
    capacity = vehicle.board_capacity
    
    if not route_node_list:
        return 0
    
    carrying_items :List[OrderItem] = (vehicle.carrying_items) if vehicle.des else []
    if not isFeasible(route_node_list , carrying_items , capacity):
        return math.inf
    
    if curr_factoryID is not None and len(curr_factoryID) > 0:
        utilTime = vehicle.leave_time_at_current_factory
        for next_node in route_node_list:
            next_factoryID = next_node.id
            distance = 0
            time= 0
            if curr_factoryID != next_factoryID:
                distance , time = route_map.get((curr_factoryID , next_factoryID))
                distance = float(distance)
                time = int(time)
                utilTime += APPROACHING_DOCK_TIME
            driving_dis += distance
            utilTime += time
            
            if next_node.delivery_item_list:
                before_orderID = ""
                next_orderID = ""
                for orderitem in next_node.delivery_item_list:
                    next_orderID = orderitem.order_id
                    if (before_orderID != next_orderID):
                        overtime_sum += max(0 , utilTime - orderitem.committed_completion_time)
                    before_orderID = next_orderID
            utilTime += next_node.service_time
            curr_factoryID = next_factoryID
    else:
        utilTime = vehicle.des.arrive_time
        curr_factoryID = route_node_list[0].id
        curr_node = route_node_list[0]
        if curr_node.delivery_item_list:
            before_orderID = ""
            next_orderID = ""
            for orderitem in curr_node.delivery_item_list:
                next_orderID = orderitem.order_id
                if (before_orderID != next_orderID):
                    overtime_sum += max(0 , utilTime - orderitem.committed_completion_time)
                before_orderID = next_orderID
        for next_node in route_node_list[1:]:
            next_factoryID = next_node.id
            distance = 0
            time = 0
            if curr_factoryID != next_factoryID:
                distance , time = route_map.get((curr_factoryID , next_factoryID))
                distance = float(distance)
                time = int(time)
                utilTime += APPROACHING_DOCK_TIME
            driving_dis += distance
            utilTime += time
            
            if next_node.delivery_item_list:
                before_orderID = ""
                next_orderID = ""
                for orderitem in next_node.delivery_item_list:
                    next_orderID = orderitem.order_id
                    if (before_orderID != next_orderID):
                        overtime_sum += max(0 , utilTime - orderitem.committed_completion_time)
                    before_orderID = next_orderID
            utilTime += next_node.service_time
            curr_factoryID = next_factoryID
    
    objF = Delta1 * overtime_sum + driving_dis
    if mode == 'distance':
        return driving_dis
    elif mode == 'overtime':
        return Delta1 * overtime_sum
    return objF

# chi phi di chuyen va overtime cua mot loi giai hoan chinh 
# 0: chi phi di chuyen
# 1: chi phi tre hang
def factorial_costs_of_an_individual(id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]]) -> List[float]:
    driving_dis  : float = 0.0
    overtime_Sum : float = 0.0
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in id_to_vehicle.items():
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
        
        if vehicleid_to_plan.get(vehicleID) and len(vehicleid_to_plan.get(vehicleID)) > 0:    
            curr_node[index] = 0
            n_node[index] = len(vehicleid_to_plan[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    driving_dis += distance
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        driving_dis += distance
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
        
    flag = False
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
        minTNodeList = vehicleid_to_plan.get(minT2VehicleID)
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
            flag = True
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]
            
        is_same_address = False
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

            is_same_address = True
            service_time += minTNodeList[curr_node[minT2VehicleIndex]].service_time
            curr_node[minT2VehicleIndex] += 1
            
        if curr_node[minT2VehicleIndex] >= n_node[minT2VehicleIndex]:
            n -= 1
            curr_node[minT2VehicleIndex] = math.inf
            curr_time[minT2VehicleIndex] = math.inf
            n_node[minT2VehicleIndex] = 0
        else:
            dis_and_time = route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                driving_dis += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    f1 : float = (driving_dis / float(len(id_to_vehicle)))
    f2 : float = Delta * overtime_Sum
    factorial_cost = [f1 , f2]
    
    return factorial_cost

def get_couple_end_idx_map(route_node_list: List[Node]):
    couple_end_idx_map = {}
    for i, node in enumerate(route_node_list):
        if node.pickup_item_list:
            for j in range(i + 1, len(route_node_list)):
                if (route_node_list[j].delivery_item_list):
                    len_delivery = len(route_node_list[j].delivery_item_list)
                    if (node.pickup_item_list[0].id ==route_node_list[j].delivery_item_list[len_delivery - 1].id):
                        couple_end_idx_map[node.pickup_item_list[0].id] = j
                        break
    return couple_end_idx_map


def dispatch_order_to_best(node_list: List[Node], cp_vehicle_id2_planned_route :Dict[str , List[Node]], id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[Tuple[str , str] , Tuple[str , str]] , mode = 'total'):
    best_insert_pos_i = -1
    best_insert_pos_j = -1 
    best_insert_vehicle_id = ''
    
    pickup_node_list = node_list[:len(node_list) // 2]
    delivery_node_list = node_list[len(node_list) // 2:]
    min_cost_delta = math.inf
    block_len = len(pickup_node_list)
    
    index = 0
    
    for vehicle_id, vehicle in id_to_vehicle.items():
        vehicle_id = f"V_{index + 1}"
        index += 1
        
        route_node_list = cp_vehicle_id2_planned_route.get(vehicle_id, [])
        cost_value0 = single_vehicle_cost(route_node_list, vehicle , route_map , mode)
        node_list_size = len(route_node_list) if route_node_list else 0
        couple_end_idx_map = get_couple_end_idx_map(route_node_list) if route_node_list else {}
        
        insert_pos = 1 if vehicle.des else 0
        
        for i in range(insert_pos, node_list_size + 1):
            temp_route_node_list = list(route_node_list) if route_node_list else []
            temp_route_node_list[i:i] = pickup_node_list
            
            for j in range(block_len + i, node_list_size + block_len + 1):
                if j == block_len + i:
                    temp_route_node_list[j:j] = delivery_node_list
                else:
                    # Neu vi tri truoc do la node nhan hang
                    if (temp_route_node_list[j - 1].pickup_item_list and len(temp_route_node_list[j - 1].pickup_item_list) > 0):
                        order_item_id = temp_route_node_list[j - 1].pickup_item_list[0].id
                        j = block_len + couple_end_idx_map.get(order_item_id, j) + 1
                        temp_route_node_list[j:j] = delivery_node_list
                    # Neu truoc do la node giao hang
                    elif (temp_route_node_list[j - 1].delivery_item_list):
                        is_terminal = True
                        for k in range(j - 2, -1, -1):
                            if (temp_route_node_list[k].pickup_item_list and temp_route_node_list[k].pickup_item_list):
                                last_delivery_item = temp_route_node_list[j - 1].delivery_item_list[-1].id
                                first_pickup_item = temp_route_node_list[k].pickup_item_list[0].id
                                if last_delivery_item == first_pickup_item:
                                    is_terminal = (k < i)
                                    break
                        if is_terminal:
                            break
                        temp_route_node_list[j:j] = delivery_node_list
                
                cost_value = single_vehicle_cost(temp_route_node_list, vehicle , route_map , mode)
                del temp_route_node_list[j:j + block_len]
                
                if (cost_value - cost_value0) < min_cost_delta:
                    min_cost_delta = cost_value - cost_value0
                    best_insert_pos_i = i
                    best_insert_pos_j = j
                    best_insert_vehicle_id = vehicle_id
    return min_cost_delta, best_insert_pos_i, best_insert_pos_j, best_insert_vehicle_id


def CHECK(temp_route_node_list: List[Node], begin_pos: int):
    route_len = len(temp_route_node_list)
    is_feasible = [[False] * route_len for _ in range(route_len)]
    
    for i in range(begin_pos, route_len - 1):
        first_node = temp_route_node_list[i]
        if first_node.delivery_item_list:
            continue
        
        for j in range(i + 1, route_len):
            last_node = temp_route_node_list[j]
            if last_node.pickup_item_list:
                continue
            
            node_list : List[Node] = []
            is_d_node_redundant = False
            
            for k in range(i, j + 1):
                node = temp_route_node_list[k]
                if node.pickup_item_list:
                    node_list.insert(0, node)
                else:
                    if (node_list) and (node_list[0].pickup_item_list) and (node_list[0].pickup_item_list[0].id == node.delivery_item_list[- 1].id):
                        node_list.pop(0)
                    else:
                        is_d_node_redundant = True
                        break
            
            if (not node_list) and (not is_d_node_redundant):
                is_feasible[i][j] = True
    
    return is_feasible

def is_overlapped(temp_route_node_list: List[Node], i: int) -> int:
    heap : List[Node] = []
    
    for k in range(i):
        if temp_route_node_list[k].pickup_item_list :
            heap.insert(0, temp_route_node_list[k])
        else:
            if heap and temp_route_node_list[k].delivery_item_list[-1].id == heap[0].pickup_item_list[0].id:
                heap.pop(0)

    idx = 0
    k = i + 2

    if heap:
        # sau hàm này thì k sẽ trả về vị trí mà chỉ đã xử lý tất cả các cặp PD đầy đủ 
        # Phần còn lại phía sau k (nếu có) chỉ là các node D và không có P (tức là đã nhận hàng)
        while k < len(temp_route_node_list):
            if temp_route_node_list[k].delivery_item_list:
                if heap[0].pickup_item_list[0].id == temp_route_node_list[k].delivery_item_list[-1].id:
                    idx = k
                    heap.pop(0)
                    if not heap or len(heap) == 0:
                        k += 1
                        break
            k += 1

    # duyệt từ vị trí đó đến hết (tất cả các node D cô độc còn lại)
    while k < len(temp_route_node_list):
        if temp_route_node_list[k].delivery_item_list:
            idx += 1
        else:
            break
        k += 1
    # trả về vị trí node D cô độc cuối cùng
    return idx

def get_block_right_bound(temp_route_node_list: List[Node], i: int) -> int:
    idx = -1

    if temp_route_node_list[i].delivery_item_list:
        return idx
    else:
        order_item0_id = temp_route_node_list[i].pickup_item_list[0].id
        for k in range(i + 1, len(temp_route_node_list)):
            if temp_route_node_list[k].delivery_item_list:
                if order_item0_id == temp_route_node_list[k].delivery_item_list[-1].id:
                    idx = k
                    break
    return idx

def get_first_p_node_idx(p_and_d_node_map: Dict[str, Node]) -> int:
    first_key = next(iter(p_and_d_node_map)) 
    p_len = len(p_and_d_node_map) // 2
    first_p_node_idx = int(first_key.split(",")[1]) - p_len + 1
    return first_p_node_idx

""" def reverse_route(temp_node_list : List[Node] , begin_pos : int , end_pos: int , vehicle: Vehicle) -> List[Node]:
    unongoing_super_node : Dict[int, Dict[str , Node]]= {}
    vehicleID = vehicle.id
    ls_nodepair_num = 0
    if temp_node_list and len(temp_node_list) > 0:
        pickup_node_heap : List[Node] = []
        pickup_node_idx_heap : List[int] = []
        p_and_d_node_map : Dict[str , Node] = {}
        idx = 0
        before_P_factoryID = None
        before_D_factoryID =None
        before_P_node_idx = 0
        before_D_node_idx = 0
        for i in range(begin_pos , end_pos + 1):
            node = temp_node_list[i]
            if node.delivery_item_list and node.pickup_item_list:
                print ("Exist combine node exception when LS" , file= sys.stderr)
                sys.exit(0)
        
            heap_top_orderitem_ID = pickup_node_heap[0].pickup_item_list[0].id if pickup_node_heap else ""
            if node.delivery_item_list:
                if node.delivery_item_list[-1].id == heap_top_orderitem_ID:
                    pickup_node_key = f"{vehicleID},{pickup_node_idx_heap[0]}"
                    delivery_node_key = f"{vehicleID},{i}"
                    
                    if len(p_and_d_node_map) >= 2:
                        if (pickup_node_heap[0].id != before_P_factoryID or pickup_node_idx_heap[0] + 1 != before_P_node_idx \
                            or node.id != before_D_factoryID or i-1 != before_D_node_idx):
                            first_pickup_node_idx = get_first_p_node_idx(p_and_d_node_map)
                            unongoing_super_node[first_pickup_node_idx] = p_and_d_node_map
                            p_and_d_node_map.clear()
                    p_and_d_node_map[pickup_node_key] = pickup_node_heap[0]
                    p_and_d_node_map[delivery_node_key] = node
                    before_P_factoryID = pickup_node_heap[0].id
                    before_P_node_idx = pickup_node_idx_heap[0]
                    before_D_factoryID = node.id
                    before_D_node_idx = i
                    pickup_node_heap.pop(0)
                    pickup_node_idx_heap.pop(0)
            if node.pickup_item_list:
                pickup_node_heap.insert(0 , node)
                pickup_node_idx_heap.insert(0 , i)
                if p_and_d_node_map:
                    first_pickup_node_idx = get_first_p_node_idx(p_and_d_node_map)
                    unongoing_super_node[first_pickup_node_idx] = p_and_d_node_map
                    p_and_d_node_map.clear()
        if len(p_and_d_node_map) >= 2:
            first_pickup_node_idx = get_first_p_node_idx(p_and_d_node_map)
            unongoing_super_node[first_pickup_node_idx] = p_and_d_node_map
    if  len(p_and_d_node_map) < 2:
            return None
        
    sorted_map = sorted(unongoing_super_node.items(), key=lambda x: x[0])

    # Xóa dữ liệu cũ và thêm dữ liệu đã sắp xếp lại vào unongoing_super_node
    unongoing_super_node.clear()
    for key, value in sorted_map:
        unongoing_super_node[key] = value
    
    before_blockI = -1
    before_blockJ = -1
    block_map :Dict[str, List[Node]]= {}
    for idx , pdg in unongoing_super_node.items():
        pickup_node : Node = None
        delivery_node : Node = None
        node_list : List[Node]= []
        pos_i = 0
        pos_j = 0
        dNum = len(pdg) // 2
        index = 0
        if pdg:
            for v_and_pos_str , node in pdg.items():
                v_and_pos_split = v_and_pos_str.split(",")
                vehicle_id = v_and_pos_split[0]
                pos = int(v_and_pos_split[1])

                if index % 2 == 0:
                    # "Pickup" node
                    pos_i = pos
                    pickup_node = node
                    node_list.insert(0, pickup_node)  # Thêm vào đầu danh sách
                    index += 1
                else:
                    # "Delivery" node
                    pos_j = pos
                    delivery_node = node
                    node_list.append(delivery_node)  # Thêm vào cuối danh sách
                    index += 1
                    pos_j = pos_j - dNum + 1
            if pos_i > before_blockJ:
                for i in range(pos_i + dNum, pos_j):
                    node_list.insert(i - pos_i, temp_node_list[i])  # Thêm các phần tử từ tempNodeList vào nodeList
                key = f"{vehicle_id},{pos_i}+{pos_j + dNum - 1}"
                block_map[key] = node_list
                before_blockI = pos_i
                before_blockJ = pos_j + dNum - 1
    # Nếu không có đủ blockMap để xử lý, trả về None
    if len(block_map) < 2:
        return None

    # Xây dựng resultNodeList từ blockMap và các phần tử còn lại trong tempNodeList
    result_node_list = []

    # Đảo ngược các phần tử trong blockMap
    reverse_block_node_list = []
    for block in block_map.values():
        reverse_block_node_list = block + reverse_block_node_list

    # Thêm phần tử từ tempNodeList trước beginPos
    result_node_list.extend(temp_node_list[:begin_pos])

    # Thêm các phần tử từ reverseBlockNodeList
    result_node_list.extend(reverse_block_node_list)

    # Thêm phần tử từ tempNodeList sau endPos
    result_node_list.extend(temp_node_list[end_pos + 1:])

    return result_node_list """

def reverse_route(temp_node_list: List[Node], begin_pos: int, end_pos: int, vehicle: Vehicle) -> Optional[List[Node]]:
    unongoing_super_node: Dict[int, Dict[str, Node]] = {}
    vehicle_id = vehicle.id

    if temp_node_list and len(temp_node_list) > 0:
        pickup_node_heap: List[Node] = []
        p_node_idx_heap: List[int] = []
        p_and_d_node_map: Dict[str, Node] = {}
        before_p_factory_id = None
        before_d_factory_id = None
        before_p_node_idx = 0
        before_d_node_idx = 0

        for i in range(begin_pos, end_pos + 1):
            node = temp_node_list[i]
            if (node.delivery_item_list and node.pickup_item_list):
                print("Exist combine Node exception when LS", file=sys.stderr)
                return None

            heap_top_order_item_id = pickup_node_heap[0].pickup_item_list[0].id if pickup_node_heap else ""
            if node.delivery_item_list:
                len_delivery = len(node.delivery_item_list)
                if len_delivery > 0 and node.delivery_item_list[-1].id == heap_top_order_item_id:
                    pickup_node_key = f"{vehicle_id},{p_node_idx_heap[0]}"
                    delivery_node_key = f"{vehicle_id},{i}"
                    if len(p_and_d_node_map) >= 2:
                        if (pickup_node_heap[0].id != before_p_factory_id or
                            p_node_idx_heap[0] + 1 != before_p_node_idx or
                            node.id != before_d_factory_id or
                            i - 1 != before_d_node_idx):
                            first_p_node_idx = get_first_p_node_idx(p_and_d_node_map)
                            unongoing_super_node[first_p_node_idx] = copy.deepcopy(p_and_d_node_map)
                            p_and_d_node_map.clear()
                    p_and_d_node_map[pickup_node_key] = pickup_node_heap.pop(0)
                    p_and_d_node_map[delivery_node_key] = node
                    before_p_factory_id = p_and_d_node_map[pickup_node_key].id
                    before_p_node_idx = p_node_idx_heap.pop(0)
                    before_d_factory_id = node.id
                    before_d_node_idx = i

            if node.pickup_item_list:
                pickup_node_heap.insert(0, node)
                p_node_idx_heap.insert(0, i)
                if p_and_d_node_map:
                    first_p_node_idx = get_first_p_node_idx(p_and_d_node_map)
                    unongoing_super_node[first_p_node_idx] = copy.deepcopy(p_and_d_node_map)
                    p_and_d_node_map.clear()

        if len(p_and_d_node_map) >= 2:
            first_p_node_idx = get_first_p_node_idx(p_and_d_node_map)
            unongoing_super_node[first_p_node_idx] =copy.deepcopy(p_and_d_node_map)

    if len(unongoing_super_node) < 2:
        return None

    # Sắp xếp unongoing_super_node theo key
    sorted_map = sorted(unongoing_super_node.items(), key=lambda x: x[0])
    unongoing_super_node.clear()
    for key, value in sorted_map:
        unongoing_super_node[key] = value

    # Xây dựng block_map
    before_block_i = -1
    before_block_j = -1
    block_map: Dict[str, List[Node]] = {}

    for idx, pdg in unongoing_super_node.items():
        node_list: List[Node] = []
        pos_i = 0
        pos_j = 0
        d_num = len(pdg) // 2
        index = 0

        if pdg:
            for v_and_pos_str, node in pdg.items():
                vehicle_id_from_key, pos_str = v_and_pos_str.split(",")
                pos = int(pos_str)
                if index % 2 == 0:  # Pickup node
                    pos_i = pos
                    node_list.insert(0, node)
                    index += 1
                else:  # Delivery node
                    pos_j = pos
                    node_list.append(node)
                    index += 1
                    pos_j = pos_j - d_num + 1

            if pos_i > before_block_j:
                for i in range(pos_i + d_num, pos_j):
                    node_list.insert(i - pos_i, temp_node_list[i])
                key = f"{vehicle_id},{pos_i}+{pos_j + d_num - 1}"
                block_map[key] = node_list
                before_block_i = pos_i
                before_block_j = pos_j + d_num - 1

    if len(block_map) < 2:
        return None

    # Đảo ngược các block và xây dựng result_node_list
    result_node_list: List[Node] = []
    reverse_block_node_list: List[Node] = []

    for block in block_map.values():
        reverse_block_node_list = block + reverse_block_node_list

    # Thêm các phần tử từ temp_node_list
    result_node_list.extend(temp_node_list[:begin_pos])
    result_node_list.extend(reverse_block_node_list)
    result_node_list.extend(temp_node_list[end_pos + 1:])

    return result_node_list

def merge_node(id_to_vehicle: Dict[str , Vehicle], vehicleid_to_plan: Dict[str, list[Node]]):
    for vehicle_id, vehicle in id_to_vehicle.items():
        origin_planned_route = vehicleid_to_plan.get(vehicle_id, [])

        if origin_planned_route and len(origin_planned_route) > 1:
            before_node = origin_planned_route[0]
            i = 1  # Bắt đầu từ phần tử thứ 2
            while (i < len(origin_planned_route)):
                next_node = origin_planned_route[i]

                if before_node.id == next_node.id:
                    # Gộp danh sách pickupItemList
                    if next_node.pickup_item_list:
                        before_node.pickup_item_list.extend(next_node.pickup_item_list)  

                    # Gộp danh sách deliveryItemList (dùng extend thay vì vòng lặp)
                    if next_node.delivery_item_list:
                        before_node.delivery_item_list.extend(next_node.delivery_item_list) 
                    # Xóa phần tử trùng lặp
                    origin_planned_route.pop(i)
                else:
                    before_node = next_node
                    i += 1  # Chỉ tăng index khi không xóa phần tử
        vehicleid_to_plan[vehicle_id] = origin_planned_route

def Delaydispatch(id_to_vehicle: Dict[str , Vehicle], vehicleid_to_plan: Dict[str, list[Node]] , route_map: Dict[tuple , tuple]):
    vehicle_num = len(id_to_vehicle)
    slack_time = 0
    emergency_index = [-1] * vehicle_num
    dock_table : Dict[str , List[List[int]]]= {} 
    n = 0 # Đếm số xe có tuyến đường hoạch định
    curr_node = [0] * vehicle_num  # Node hiện tại trong bối cảnh của hàm
    curr_time = [0] * vehicle_num # Thời gian trong bối cảnh của hàm
    n_node = [0] * vehicle_num # số node trong tuyến đường hoạch định của xe
    index = 0 # Đếm số xe

    # Xét từng xe
    for vehicle_id, vehicle in id_to_vehicle.items():
        planned_route = vehicleid_to_plan.get(vehicle_id , [])
        
        # Nếu có tuyến đường được hoạch định
        if planned_route and len(planned_route) > 0:
            curr_node[index] = 0
            n_node[index] = len(planned_route)

            # Nếu phương tiện chưa có đích đến <-> xe đang đỗ
            if vehicle.des is None:
                if vehicle.cur_factory_id == planned_route[0].id:
                    curr_time[index] = vehicle.gps_update_time
                else:
                    dis_and_time = route_map.get((vehicle.cur_factory_id , planned_route[0].id))
                    if dis_and_time is None:
                        print(f"No distance found for route", file = sys.stderr)
                    else:
                        curr_time[index] = vehicle.gps_update_time + int(dis_and_time[1])
            # Xe đang di chuyển
            else:
                if vehicle.cur_factory_id and len(vehicle.cur_factory_id) > 0:
                    if vehicle.cur_factory_id != planned_route[0].id:
                        curr_time[index] = vehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((vehicle.cur_factory_id , planned_route[0].id))
                        if dis_and_time:
                            curr_time[index] += int(dis_and_time[1])
                    else:
                        curr_time[index] = vehicle.leave_time_at_current_factory
                else:
                    curr_time[index] = vehicle.des.arrive_time
            
            n += 1
        else:
            curr_node[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
    
    flag = False
    # Xét tất cả các xe có tuyến đường được hoạch định
    while n > 0:
        min_t = math.inf
        min_t2_vehicle_index = 0
        t_true = min_t
        idx = 0

        # Tìm xe có thời gian (đến node gần nhất) nhỏ nhất
        for i in range(vehicle_num):
            if curr_time[i] < min_t:
                min_t = curr_time[i]
                min_t2_vehicle_index = i

        # Mã của xe có thời gian nhỏ nhất
        min_t2_vehicle_id = f"V_{min_t2_vehicle_index + 1}"
        # Tuyến đường được hoạch định của xe
        min_t_node_list = vehicleid_to_plan.get(min_t2_vehicle_id)
        min_t_node = min_t_node_list[curr_node[min_t2_vehicle_index]]

        if min_t_node.delivery_item_list:
            for order_item in min_t_node.delivery_item_list:
                commit_complete_time = order_item.committed_completion_time
                slack_time = commit_complete_time - min_t
                if slack_time < SLACK_TIME_THRESHOLD:
                    emergency_index[min_t2_vehicle_index] = curr_node[min_t2_vehicle_index]
                    break

        used_end_time = []
        time_slots : List[List[int]]= dock_table.get(min_t_node.id, [])

        if time_slots:
            i = 0
            while i < len(time_slots):
                timeslot = time_slots[i]
                if timeslot[1] <= min_t:
                    del time_slots[i]
                    i -=1 
                elif timeslot[0] <= min_t and min_t < timeslot[1]:
                    used_end_time.append(timeslot[1])
                else:
                    print("------------ timeslot.start>minT--------------", file = sys.stderr)
                i += 1

        if len(used_end_time) < 6:
            t_true = min_t
        else:
            flag = True
            idx = len(used_end_time) - 6
            used_end_time = sorted(used_end_time)
            t_true = used_end_time[idx]

        service_time = min_t_node_list[curr_node[min_t2_vehicle_index]].service_time
        cur_factory_id = min_t_node_list[curr_node[min_t2_vehicle_index]].id
        curr_node[min_t2_vehicle_index] += 1

        while ((curr_node[min_t2_vehicle_index] < n_node[min_t2_vehicle_index]) and ( cur_factory_id == min_t_node_list[curr_node[min_t2_vehicle_index]].id)):

            if min_t_node_list[curr_node[min_t2_vehicle_index]].delivery_item_list:
                for order_item in min_t_node_list[curr_node[min_t2_vehicle_index]].delivery_item_list:
                    commit_complete_time = order_item.committed_completion_time
                    slack_time = commit_complete_time - min_t
                    if slack_time < SLACK_TIME_THRESHOLD:
                        emergency_index[min_t2_vehicle_index] = curr_node[min_t2_vehicle_index]
                        break

            service_time += min_t_node_list[curr_node[min_t2_vehicle_index]].service_time
            curr_node[min_t2_vehicle_index] += 1

        if curr_node[min_t2_vehicle_index] >= n_node[min_t2_vehicle_index]:
            n -= 1
            curr_node[min_t2_vehicle_index] = float('inf')
            curr_time[min_t2_vehicle_index] = float('inf')
            n_node[min_t2_vehicle_index] = 0
        else:
            dis_and_time = route_map.get((cur_factory_id , min_t_node_list[curr_node[min_t2_vehicle_index]].id))
            time = int(dis_and_time[1])
            curr_time[min_t2_vehicle_index] = t_true + APPROACHING_DOCK_TIME + service_time + time


        tw = [min_t, t_true + APPROACHING_DOCK_TIME + service_time]
        twList = dock_table.get(min_t_node.id , [])
        twList.append(tw)
        dock_table[min_t_node.id] = twList
    
    idx = 0
    for vehicleID , node_list in vehicleid_to_plan.items():
        vehicle = id_to_vehicle.get(vehicleID)
        
        if node_list and (emergency_index[idx] > -1 or vehicle.carrying_items or vehicle.des):
            delivery_item_list : List[OrderItem] = []
            carrying_items_list = vehicle.carrying_items
            if carrying_items_list:
                i = len(carrying_items_list) - 1
                while i >= 0:
                    delivery_item_list.append(carrying_items_list[i])
                    i -=1
            
            for k in range(len(node_list)):
                if emergency_index[idx] <= -1:
                    break
                
                node = node_list[k]
                delivery_items = node.delivery_item_list
                pickup_items = node.pickup_item_list
                
                if delivery_items and k <= emergency_index[idx]:
                    for order_item in delivery_items:
                        if (not delivery_item_list) or len(delivery_item_list) == 0 or delivery_item_list[0].id != order_item.id:
                            print("violate FILO _ DelayDispatch", file=sys.stderr)
                        delivery_item_list.pop(0)
                
                if pickup_items and k <= emergency_index[idx]:
                    for order_item in pickup_items:
                        delivery_item_list.insert(0, order_item)

            is_des_empty = True
            if vehicle.des and len(delivery_item_list) == 0:
                is_des_empty = False

            e = emergency_index[idx]
            if delivery_item_list or (not is_des_empty):
                for i in range(e + 1, len(node_list)):
                    node = node_list[i]
                    
                    if node.delivery_item_list:
                        for order_item in node.delivery_item_list:
                            if order_item in delivery_item_list:
                                delivery_item_list.remove(order_item)

                    if node.pickup_item_list:
                        for order_item in node.pickup_item_list:
                            delivery_item_list.insert(0, order_item)

                    emergency_index[idx] = i
        idx += 1
    return emergency_index


def write_destination_json_to_file_with_delay_timme(vehicleid_to_destination : Dict[str , Node] , emergency_index: List[int] , id_to_vehicle: Dict[str , Vehicle] , input_directory: str):
    result_json = {}
    
    for index , (vehicleID , des) in enumerate(vehicleid_to_destination.items()):
        current_node = None
        if emergency_index[index] > -1 or id_to_vehicle.get(vehicleID).des:
            if des:
                pickup_items = []
                delivery_items = []
                current_node ={}
                
                if des.pickup_item_list and len(des.pickup_item_list) > 0:
                    for orderitem in des.pickup_item_list:
                        pickup_items.append(orderitem.id)
                        
                if des.delivery_item_list and len(des.delivery_item_list) > 0:
                    for orderitem in des.delivery_item_list:
                        delivery_items.append(orderitem.id)
                
                current_node = {
                    "factory_id": des.id,
                    "lng": des.lng,
                    "lat": des.lat,
                    "delivery_item_list": delivery_items,
                    "pickup_item_list": pickup_items,
                    "arrive_time": des.arrive_time,
                    "leave_time": des.leave_time
                }
        result_json[vehicleID] = current_node
    
      # Đảm bảo input_directory hợp lệ
    if not os.path.isdir(input_directory):
        try:
            os.makedirs(input_directory, exist_ok=True)
        except OSError as e:
            print(f"Lỗi khi tạo thư mục: {e}", file = sys.stderr)
            return  # Tránh tiếp tục nếu có lỗi

    output_file = os.path.join(input_directory, "output_destination.json")

    # Ghi dữ liệu ra file JSON với kiểm soát lỗi
    try:
        with open(output_file, "w", encoding="utf-8") as file:
            json.dump(result_json, file, ensure_ascii=False, indent=4)
    except IOError as e:
        print(f"Lỗi khi ghi file JSON: {e}", file = sys.stderr)

def write_destination_json_to_file(vehicleid_to_destination : Dict[str , Node] , id_to_vehicle: Dict[str , Vehicle] , input_directory: str):
    result_json = {}
    
    for index , (vehicleID , des) in enumerate(vehicleid_to_destination.items()):
        current_node = None
        if des:
            pickup_items = []
            delivery_items = []
            current_node ={}
            
            if des.pickup_item_list and len(des.pickup_item_list) > 0:
                for orderitem in des.pickup_item_list:
                    pickup_items.append(orderitem.id)
                    
            if des.delivery_item_list and len(des.delivery_item_list) > 0:
                for orderitem in des.delivery_item_list:
                    delivery_items.append(orderitem.id)
            
            current_node = {
                "factory_id": des.id,
                "lng": des.lng,
                "lat": des.lat,
                "delivery_item_list": delivery_items,
                "pickup_item_list": pickup_items,
                "arrive_time": des.arrive_time,
                "leave_time": des.leave_time
            }
        result_json[vehicleID] = current_node
    
      # Đảm bảo input_directory hợp lệ
    if not os.path.isdir(input_directory):
        try:
            os.makedirs(input_directory, exist_ok=True)
        except OSError as e:
            print(f"Lỗi khi tạo thư mục: {e}", file = sys.stderr)
            return  # Tránh tiếp tục nếu có lỗi

    output_file = os.path.join(input_directory, "output_destination.json")

    # Ghi dữ liệu ra file JSON với kiểm soát lỗi
    try:
        with open(output_file, "w", encoding="utf-8") as file:
            json.dump(result_json, file, ensure_ascii=False, indent=4)
    except IOError as e:
        print(f"Lỗi khi ghi file JSON: {e}", file = sys.stderr)


def write_route_json_to_file_with_delay_time(vehicleid_to_plan: Dict[str, list[Node]] ,  emergency_index: List[int] , id_to_vehicle: Dict[str , Vehicle] , input_directory: str):
    result_json = {}
    
    for index , (vehicleID , nodelist) in enumerate(vehicleid_to_plan.items()):
        vehicle_items = []
        
        if emergency_index[index] > -1:
            if nodelist:
                for i in range(0 ,emergency_index[index]):
                    node  = nodelist[i]
                    pickup_items = []
                    delivery_items = []
                    current_node ={}
                    
                    if node.pickup_item_list and len(node.pickup_item_list) > 0:
                        for orderitem in node.pickup_item_list:
                            pickup_items.append(orderitem.id)
                            
                    if node.delivery_item_list and len(node.delivery_item_list) > 0:
                        for orderitem in node.delivery_item_list:
                            delivery_items.append(orderitem.id)
                    
                    current_node = {
                        "factory_id": node.id,
                        "lng": node.lng,
                        "lat": node.lat,
                        "delivery_item_list": delivery_items,
                        "pickup_item_list": pickup_items,
                        "arrive_time": node.arrive_time,
                        "leave_time": node.leave_time
                    }
                    vehicle_items.append(current_node)
        result_json[vehicleID] = vehicle_items
        
    # Đảm bảo thư mục đầu ra hợp lệ
    if not os.path.isdir(input_directory):
        try:
            os.makedirs(input_directory, exist_ok=True)
        except OSError as e:
            print(f"Lỗi khi tạo thư mục: {e}", file = sys.stderr)
            return  # Tránh tiếp tục nếu có lỗi

    output_file = os.path.join(input_directory, "output_route.json")

    # Ghi dữ liệu ra file JSON với kiểm soát lỗi
    try:
        with open(output_file, "w", encoding="utf-8") as file:
            json.dump(result_json, file, ensure_ascii=False, indent=4)
    except IOError as e:
        print(f"Lỗi khi ghi file JSON: {e}", file = sys.stderr)


def write_route_json_to_file(vehicleid_to_plan: Dict[str, list[Node]] , id_to_vehicle: Dict[str , Vehicle] , input_directory: str):
    result_json = {}
    
    for index , (vehicleID , nodelist) in enumerate(vehicleid_to_plan.items()):
        vehicle_items = []
        if nodelist:
            for i in range(0 , len(nodelist)):
                node  = nodelist[i]
                pickup_items = []
                delivery_items = []
                current_node ={}
                
                if node.pickup_item_list and len(node.pickup_item_list) > 0:
                    for orderitem in node.pickup_item_list:
                        pickup_items.append(orderitem.id)
                        
                if node.delivery_item_list and len(node.delivery_item_list) > 0:
                    for orderitem in node.delivery_item_list:
                        delivery_items.append(orderitem.id)
                
                current_node = {
                    "factory_id": node.id,
                    "lng": node.lng,
                    "lat": node.lat,
                    "delivery_item_list": delivery_items,
                    "pickup_item_list": pickup_items,
                    "arrive_time": node.arrive_time,
                    "leave_time": node.leave_time
                }
                vehicle_items.append(current_node)
        result_json[vehicleID] = vehicle_items
        
    # Đảm bảo thư mục đầu ra hợp lệ
    if not os.path.isdir(input_directory):
        try:
            os.makedirs(input_directory, exist_ok=True)
        except OSError as e:
            print(f"Lỗi khi tạo thư mục: {e}", file = sys.stderr)
            return  # Tránh tiếp tục nếu có lỗi

    output_file = os.path.join(input_directory, "output_route.json")

    # Ghi dữ liệu ra file JSON với kiểm soát lỗi
    try:
        with open(output_file, "w", encoding="utf-8") as file:
            json.dump(result_json, file, ensure_ascii=False, indent=4)
    except IOError as e:
        print(f"Lỗi khi ghi file JSON: {e}", file = sys.stderr)

def copy_solution_file(input_directory):
    """Sao chép nội dung solution.json sang thư mục solution/ với tên file mới."""
    
    # Đường dẫn file nguồn
    source_path = os.path.join(input_directory, "solution.json")

    # Đọc dữ liệu từ file nguồn
    try:
        with open(source_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
    except (IOError, json.JSONDecodeError) as e:
        print(f"Lỗi khi đọc file {source_path}: {e}", file = sys.stderr)
        return False  # Trả về False nếu không thể đọc file
    
    # Tạo tên file mới từ 'no' và 'deltaT', mặc định là 'unknown' nếu không có giá trị
    no = data.get('no.', 'unknown')
    deltaT = data.get('deltaT', 'unknown')
    new_file_name = f"{no}_{deltaT}.json"  # Định dạng tên file

    # Đường dẫn thư mục đích
    dest_dir = os.path.join(input_directory, 'solution')
    os.makedirs(dest_dir, exist_ok=True)  # Đảm bảo thư mục tồn tại

    # Đường dẫn file đích
    dest_path = os.path.join(dest_dir, new_file_name)

    # Ghi dữ liệu vào file mới
    try:
        with open(dest_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
    except IOError as e:
        print(f"Lỗi khi ghi file {dest_path}: {e}", file = sys.stderr)
        return False  # Trả về False nếu có lỗi khi ghi file

    return True  # Thành công


def deal_old_solution_file(id2VehicleMap):
    # Xác định thời gian 00:00:00 của ngày hiện tại (UNIX timestamp, tính theo giây)
    now = datetime.now()
    initial_time = int(datetime(now.year, now.month, now.day, 0, 0, 0).timestamp())

    # Kiểm tra điều kiện thời gian cập nhật GPS
    if id2VehicleMap["V_1"].gps_update_time - 600 == initial_time:
        # Kiểm tra nếu bất kỳ xe nào có điểm đến thì thoát
        for vehicle in id2VehicleMap.values():
            if vehicle.des is not None:
                return

        # Xóa file solution.json nếu tồn tại
        file_path = "./algorithm/data_interaction/solution.json"
        if os.path.exists(file_path):
            os.remove(file_path)
            
def get_route_after(vehicleid_to_plan: Dict[str , list[Node]], vehicleid_to_destination : Dict[str , Node]):
    
    route_str = ""
    vehicle_num = len(vehicleid_to_plan)
    vehicle_routes = [""] * vehicle_num
    index = 0
    
    if vehicleid_to_destination is None or len(vehicleid_to_destination) == 0:
        for i in range(vehicle_num):
            vehicle_routes[i] = "["
    for vehicle_id, first_node in vehicleid_to_destination.items():
        if first_node is not None:
            pickup_size = len(first_node.pickup_item_list) if first_node.pickup_item_list else 0
            delivery_size = len(first_node.delivery_item_list) if first_node.delivery_item_list else 0
            
            if delivery_size > 0:
                vehicle_routes[index] = f"[d{delivery_size}_{first_node.delivery_item_list[0].id} "
            if pickup_size > 0:
                if delivery_size == 0:
                    vehicle_routes[index] = f"[p{pickup_size}_{first_node.pickup_item_list[0].id} "
                else:
                    vehicle_routes[index] = vehicle_routes[index].strip()
                    vehicle_routes[index] += f"p{pickup_size}_{first_node.pickup_item_list[0].id} "
        else:
            vehicle_routes[index] = "["
        index += 1
    
    index = 0
    for vehicle_id, id2_node_list in vehicleid_to_plan.items():
        if id2_node_list and len(id2_node_list) > 0:
            for node in id2_node_list:
                pickup_size = len(node.pickup_item_list)
                delivery_size = len(node.delivery_item_list)
                
                if delivery_size > 0:
                    vehicle_routes[index] += f"d{delivery_size}_{node.delivery_item_list[0].id} "
                if pickup_size > 0:
                    if delivery_size > 0:
                        vehicle_routes[index] = vehicle_routes[index].strip()
                    vehicle_routes[index] += f"p{pickup_size}_{node.pickup_item_list[0].id} "
            
            vehicle_routes[index] = vehicle_routes[index].strip()
        vehicle_routes[index] += "]"
        index += 1

    for i in range(vehicle_num):
        car_id = f"V_{i + 1}"
        route_str += f"{car_id}:{vehicle_routes[i]} "
    
    route_str = route_str.strip()
    return route_str
