from datetime import datetime 
import json
import os
import sys
from typing import Dict , List
from algorithm.GA import GA
from algorithm.CCEA import CCEA
from algorithm.Read_input import Input
from algorithm.Object import *
import copy
import time
from algorithm.local_search import *
from algorithm.local_search2 import *
import re


input_directory = r'algorithm\data_interaction'
delta_t = "0000-0010"
before_cost = 0.0
completeOrderItems: str = ""
newOrderItems: str = ""
onVehicleOrderItems : str = ""
unallocatedOrderItems: str = ""
routeBefore: str = ""
used_time = 0.0
n1, n2, n3, n4, n5 = 0, 0, 0, 0, 0


def restore_scene_with_single_node(vehicleid_to_plan: Dict[str , List[Node]], id_to_ongoing_items: Dict[str , OrderItem], id_to_unlocated_items: Dict[str , OrderItem], id_to_vehicle: Dict[str , Vehicle] , id_to_factory: Dict[str , Factory], id_to_allorder: Dict[str , OrderItem]) -> List[str]:
    global before_cost, delta_t , completeOrderItems , newOrderItems , onVehicleOrderItems , unallocatedOrderItems , routeBefore
    global n1, n2, n3, n4, n5
    new_order_itemIDs = []
    
    for vehicleID in id_to_vehicle.keys():
        vehicleid_to_plan[vehicleID] = []
    
    for key in id_to_ongoing_items:
        onVehicleOrderItems += f"{key} "
    onVehicleOrderItems.strip()
    
    for key in id_to_unlocated_items:
        unallocatedOrderItems += f"{key} "
    unallocatedOrderItems.strip()
    
    solution_json_path = os.path.join(input_directory , 'solution.json')
    if os.path.exists(solution_json_path) :
        try:
            with open(solution_json_path , 'r') as file:
                before_solution = json.load(file)
                no = int(before_solution.get('no', 0))
                f = (no + 1) * 10
                t = (no + 1) * 10 + 10
                global delta_t
                delta_t = f"{f:04d}-{t:04d}"
                routeBefore = before_solution.get("route_after", "")
                splited_routeBefore : List[str] = routeBefore.split("V")
                
                last_on_vehicle_items= before_solution.get("onvehicle_order_items", "").split()
                curr_on_vehicle_items : List[str] = onVehicleOrderItems.split(" ")
                completeOrderItems = ' '.join([item for item in last_on_vehicle_items if item not in curr_on_vehicle_items]).strip()
                complete_item_array = completeOrderItems.split(" ")
                
                last_unallocated_items : List[str] = before_solution.get("unallocated_order_items", "").split()
                curr_unallocated_items : List[str] = unallocatedOrderItems.split(" ")
                newOrderItems = ' '.join([item for item in curr_unallocated_items if item not in last_unallocated_items]).strip()
                
                for route in splited_routeBefore:
                    if not route or len(route) < 3:
                        continue
                    
                    route = route.strip()
                    str_len : int = len(route.split(':')[1])
                    numstr = route.split(":")[0]
                    vehicleID = "V_" + numstr[1:]
                    if str_len < 3: 
                        vehicleid_to_plan[vehicleID] = []
                        continue
                    
                    route_nodes_str = route.split(":")[1]
                    route_nodes = route_nodes_str[1:len(route_nodes_str) - 1].split(" ")
                    node_list : List[str] = list(route_nodes)
                    
                    # bao gồm các node (đại diện bởi itemID) cò tới thời điểm của time interval hiện tại
                    node_list = [
                        node for node in node_list
                        if not (
                            (node.startswith("d") and node.split("_")[1] in complete_item_array) or
                            (node.startswith("p") and node.split("_")[1] in curr_on_vehicle_items)
                        )
                    ]
                    
                    if len(node_list) > 0:
                        planroute : List[Node] = []
                        
                        for node in node_list:
                            deliveryItemList : List[OrderItem] = []
                            pickupItemList : List[OrderItem] = []
                            temp : OrderItem = None
                            op = node[0][0:1]           #chỉ thị trạng thái của node (pickup / delivery) (p/d)
                            opNumstr = node.split("_")
                            opItemNum = int(opNumstr[0][1 :]) #p3 -> 3
                            orderItemID = node.split("_")[1]
                            idEndNumber = int(orderItemID.split("-")[1]) #số hiệu lớn nhất của đơn hàng
                            
                            # nếu là node giao
                            if op == 'd':
                                for i in range(opItemNum):
                                    temp = id_to_allorder[orderItemID]
                                    deliveryItemList.append(temp)
                                    
                                    idEndNumber -= 1
                                    orderItemID =  orderItemID.split("-")[0] + "-" + str(idEndNumber)
                            # nếu là node nhận
                            else:
                                for i in range(opItemNum):
                                    temp = id_to_allorder[orderItemID]
                                    pickupItemList.append(temp)
                                    
                                    idEndNumber += 1
                                    orderItemID =  orderItemID.split("-")[0] + "-" + str(idEndNumber)
                            
                            factoryID = ""
                            if op == 'd':
                                factoryID = temp.delivery_factory_id
                            else:
                                factoryID = temp.pickup_factory_id
                            factory = id_to_factory[factoryID]
                            
                            planroute.append(Node(factoryID , deliveryItemList , pickupItemList ,None ,None , factory.lng , factory.lat))
                            
                        if len(planroute) > 0:
                            vehicleid_to_plan[vehicleID] = planroute
                LC_count = before_solution.get("Local_search", "")
                numbers = list(map(int, re.findall(r'(?<=:)\d+', LC_count)))
                n1, n2, n3, n4, n5 = numbers
        except Exception as e:
            print(f"Error: {e}" , file= sys.stderr)
    else:
        newOrderItems  = unallocatedOrderItems
        completeOrderItems = ""
        routeBefore = ""
        delta_t = "0000-0010"
        
    new_order_itemIDs = newOrderItems.split()
    if os.path.exists(solution_json_path) :
        try:
            with open(solution_json_path , 'r') as file:
                before_solution = json.load(file)

                last_route_before : str = before_solution.get('route_before' , '')
                last_route_after : str = before_solution.get('route_after' , '')
                last_ongoinging_items : List[str] = before_solution.get('ongoing_order_items' , '').split()
                last_unongoinging_items : List[str] = before_solution.get('unongoing_order_items' , '').split()
                
                last_unallocated_items : List[str] = before_solution.get("unallocated_order_items", "").split()
                
                for itemID  in last_unallocated_items:
                    if (itemID in last_ongoinging_items) or (itemID in last_unongoinging_items):
                        orderID =  str(itemID.split('-')[0])
                        temp1 = last_route_before.count(orderID)
                        temp2 = last_route_after.count(orderID)
                        if temp1 > 0 and temp2 == 0:
                            new_order_itemIDs.append(itemID)
        except Exception as e:
            print(f"Error: {e}" , file= sys.stderr)
    
    return new_order_itemIDs


def dispatch_new_orders(vehicleid_to_plan: Dict[str , list[Node]] ,  id_to_factory:Dict[str , Factory] , route_map: Dict[tuple , tuple] ,  id_to_vehicle: Dict[str , Vehicle] , id_to_unlocated_items:Dict[str , OrderItem], new_order_itemIDs: list[str]):
    if new_order_itemIDs:
        orderId_to_Item : Dict[str , list[OrderItem]] = {}
        for new_order_item in new_order_itemIDs:
            new_item = id_to_unlocated_items.get(new_order_item)
            orderID  = new_item.order_id
            if orderID not in orderId_to_Item:
                orderId_to_Item[orderID] = []
            orderId_to_Item.get(orderID).append(new_item)
        
        for vehicle in id_to_vehicle.values():
            capacity = vehicle.board_capacity
            break
        
        for orderID , orderID_items in orderId_to_Item.items():
            order_demand = 0
            for item in orderID_items:
                order_demand += item.demand
            
            if order_demand > capacity:
                tmp_demand = 0
                tmp_itemList: list[OrderItem] = []
                for item in orderID_items:
                    if (tmp_demand + item.demand) > capacity:
                        node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                        isExhausive = False
                        route_node_list : List[Node] = []
                        
                        if node_list:
                            isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan , route_map)
                        
                        route_node_list = vehicleid_to_plan.get(bestInsertVehicleID , [])

                        if isExhausive:
                            route_node_list = bestNodeList[:]
                        else:
                            if route_node_list is None:
                                route_node_list = []
                            
                            new_order_pickup_node = node_list[0]
                            new_order_delivery_node = node_list[1]
                            
                            route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                            route_node_list.insert(bestInsertPosJ, new_order_delivery_node)
                        vehicleid_to_plan[bestInsertVehicleID] = route_node_list
                        
                        tmp_itemList.clear()
                        tmp_demand = 0
                    tmp_itemList.append(item)
                    tmp_demand += item.demand 

                if len(tmp_itemList) > 0:
                    node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                    isExhausive = False
                    
                    if node_list:
                        isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList =  dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan, route_map)
                    route_node_list : List[Node] = vehicleid_to_plan.get(bestInsertVehicleID , [])
                    
                    if isExhausive:
                        route_node_list = bestNodeList[:]
                    else:
                        if route_node_list is None:
                            route_node_list = []
                        
                        new_order_pickup_node = node_list[0]
                        new_order_delivery_node = node_list[1]
                        
                        route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                        route_node_list.insert(bestInsertPosJ, new_order_delivery_node)
                    vehicleid_to_plan[bestInsertVehicleID] = route_node_list
            else:
                node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(orderID_items) , id_to_factory)
                
                isExhausive = False
                if node_list:
                    isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan , route_map)
                route_node_list : List[Node] = vehicleid_to_plan.get(bestInsertVehicleID , [])
                if isExhausive:
                    route_node_list = bestNodeList[:]
                else:
                    if route_node_list is None:
                        route_node_list = []
                    
                    new_order_pickup_node = node_list[0]
                    new_order_delivery_node = node_list[1]
                    
                    route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                    route_node_list.insert(bestInsertPosJ, new_order_delivery_node)
                vehicleid_to_plan[bestInsertVehicleID] = route_node_list


def update_solution_json (id_to_ongoing_items: Dict[str , OrderItem] , id_to_unlocated_items: Dict[str , OrderItem] , id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str , list[Node]] , vehicleid_to_destination : Dict[str , Node] , route_map: Dict[tuple , tuple]):
    global  input_directory ,delta_t , used_time
    order_items_json_path = os.path.join(input_directory, "solution.json")
    complete_order_items = ""
    on_vehicle_order_items = ""
    ongoing_order_items = ""
    unongoing_order_items = ""
    unallocated_order_items = ""
    new_order_items = ""
    route_before = ""
    route_after = ""
    solution_json_obj = {}

    on_vehicle_order_items = " ".join(id_to_ongoing_items.keys()).strip()

    unallocated_order_items = " ".join(id_to_unlocated_items.keys()).strip()

    os.makedirs(input_directory, exist_ok=True)
    
    pre_matching_item_ids = []
    for vehicle in id_to_vehicle.values():
        if (not vehicle.carrying_items) and  (vehicle.des) :
            pickup_item_list : List[OrderItem] = vehicle.des.pickup_item_list
            pre_matching_item_ids.extend([order_item.id for order_item in pickup_item_list])
    ongoing_order_items = " ".join(pre_matching_item_ids).strip()

    unallocated_items = unallocated_order_items.split()
    unongoing_order_items = " ".join([item for item in unallocated_items if item not in pre_matching_item_ids]).strip()

    if not os.path.exists(order_items_json_path):
        delta_t = "0000-0010"
        vehicle_num = len(vehicleid_to_plan)
        for i in range(vehicle_num):
            car_id = f"V_{i + 1}"
            route_before += f"{car_id}:[] "
        route_before = route_before.strip()

        route_after = get_route_after(vehicleid_to_plan , vehicleid_to_destination)
        
        solution_json_obj = {
            "no.": "0",
            "deltaT": delta_t,
            "complete_order_items": complete_order_items,
            "onvehicle_order_items": on_vehicle_order_items,
            "ongoing_order_items": ongoing_order_items,
            "unongoing_order_items": unongoing_order_items,
            "unallocated_order_items": unallocated_order_items,
            "new_order_items": unallocated_order_items,
            "used_time": used_time,
            "finalCost": total_cost(id_to_vehicle , route_map , vehicleid_to_plan),
            "route_before": route_before,
            "route_after": route_after,
            "Local_search": f'PDPairExchange:{n1}; BlockExchange:{n2}; BlockRelocate:{n3}; mPDG:{n4}; 2opt:{n5}',
        }
    else:
        try:
            with open(order_items_json_path, 'r', encoding='utf-8') as file:
                before_solution = json.load(file)
        except (IOError, json.JSONDecodeError) as e:
            print(f"Lỗi khi đọc file JSON: {e}", file = sys.stderr)
            return  # Ngăn lỗi tiếp tục chạy

        no = int(before_solution["no."]) + 1

        from_t = (no + 1) * 10
        to_t = (no + 1) * 10 + 10
        from_t_str = f"{from_t:04d}" if from_t < 10000 else str(from_t)
        to_t_str = f"{to_t:04d}" if to_t < 10000 else str(to_t)
        delta_t = f"{from_t_str}-{to_t_str}"

        last_onvehicle_order_item = before_solution["onvehicle_order_items"].split()
        curr_onvehicle_order_item = on_vehicle_order_items.split()
        complete_order_items = ' '.join([item for item in last_onvehicle_order_item if item not in curr_onvehicle_order_item]).split()
        
        last_unallocated_items = before_solution.get("unallocated_order_items", "").split()
        cur_unallocated_items = unallocated_order_items.split()
        new_order_items = " ".join([item for item in cur_unallocated_items if item not in last_unallocated_items]).split()


        route_before = before_solution["route_after"]
        route_after = get_route_after(vehicleid_to_plan , vehicleid_to_destination)

        solution_json_obj = {
            "no.": str(no),
            "deltaT": delta_t,
            "complete_order_items": complete_order_items,
            "onvehicle_order_items": on_vehicle_order_items,
            "ongoing_order_items": ongoing_order_items,
            "unongoing_order_items": unongoing_order_items,
            "unallocated_order_items": unallocated_order_items,
            "new_order_items": new_order_items,
            "used_time": used_time,
            "finalCost": total_cost(id_to_vehicle , route_map , vehicleid_to_plan),
            "route_before": route_before,
            "route_after": route_after,
            "Local_search": f'PDPairExchange:{n1}; BlockExchange:{n2}; BlockRelocate:{n3}; mPDG:{n4}; 2opt:{n5}'
        }

    # Ghi dữ liệu ra file JSON
    try:
        with open(order_items_json_path, 'w', encoding='utf-8') as file:
            json.dump(solution_json_obj, file, indent=4, ensure_ascii=False)
    except IOError as e:
        print(f"Lỗi khi ghi file JSON: {e}", file = sys.stderr)


def get_output_solution(id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str , list[Node]] , vehicleid_to_destination : Dict[str , Node]):
    for vehicleID , vehicle in id_to_vehicle.items():
        origin_plan : List[Node]= vehicleid_to_plan.get(vehicleID , [])
        destination : Node = None
        if vehicle.des:
            if (not origin_plan):
                print(f"Planned route of vehicle {vehicleID} is wrong", file=sys.stderr)
            else:
                destination = origin_plan[0]
                destination.arrive_time = vehicle.des.arrive_time
                origin_plan.pop(0)
            
            if destination and vehicle.des.id != destination.id:
                print(f"Vehicle {vehicleID} returned destination id is {vehicle.des.id} "
                    f"however the origin destination id is {destination.id}", file=sys.stderr)
        elif (origin_plan):
            destination = origin_plan[0]
            origin_plan.pop(0)
        if origin_plan and len(origin_plan) == 0:
            origin_plan = None
        vehicleid_to_plan[vehicleID] = origin_plan
        vehicleid_to_destination[vehicleID] = destination


def over24hours(id_to_vehicle : Dict[str , Vehicle]):
    global newOrderItems
    now = datetime.now()
    midnight = datetime(now.year, now.month, now.day)

    initial_time = int(midnight.timestamp())  

    # Kiểm tra điều kiện
    if (id_to_vehicle.get("V_1").gps_update_time - 600 - 86400 >= initial_time and len(newOrderItems) == 0):
        return True
    return False


def redispatch_process(id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]] ,  id_to_factory:Dict[str , Factory] , id_to_unlocated_items:Dict[str , OrderItem] ):
    global newOrderItems
    cost0 = total_cost(id_to_vehicle , route_map , vehicleid_to_plan)
    order_item_ids = []
    backup_restore_solution : Dict[str,List[Node]]= {}
    for idx , (vehicleID , nodelist) in enumerate(vehicleid_to_plan.items()):
        vehicle = id_to_vehicle.get(vehicleID)
        route_size = len(nodelist) if nodelist else 0
        begin_pos = 1 if vehicle.des else 0
        
        if route_size == 0:
            backup_restore_solution[vehicleID] = []
            continue
        
        cp_nodelist = copy.deepcopy(nodelist)
        backup_restore_solution[vehicleID] = cp_nodelist
        print(len(nodelist) , file= sys.stderr)
        
        i = begin_pos
        while i < len(nodelist):
            node1 = nodelist[i]
            if node1.pickup_item_list:  # Kiểm tra không rỗng
                begin_item_id = node1.pickup_item_list[0].id
                
                j = i + 1
                while j < len(nodelist):
                    node2 = nodelist[j]
                    
                    if node2.delivery_item_list and node2.delivery_item_list[-1].id == begin_item_id:
                        for item in node1.pickup_item_list:
                            order_item_ids.append(item.id)
                        
                        del nodelist[j]   # Xóa node2 trước
                        del nodelist[i]   # Xóa node1 sau

                        i -= 1  # Giảm i để kiểm tra lại vị trí hiện tại
                        break  # Thoát vòng lặp j
                    j += 1
            i += 1  # Chỉ tăng i nếu không xóa phần tử

    if order_item_ids:
        order_item_ids.sort(key=lambda x: (re.split(r"-", x)[0], int(re.split(r"-", x)[1])))
        newOrderItems =" " + " ".join(order_item_ids)
        newOrderItems = newOrderItems.strip()
    
    new_order_itemIDs = newOrderItems.split(" ")
    new_order_itemIDs = [item for item in new_order_itemIDs if item]
    
    #print(new_order_itemIDs , file= sys.stderr)
    dispatch_new_orders(vehicleid_to_plan , id_to_factory , route_map , id_to_vehicle , id_to_unlocated_items , new_order_itemIDs)
    
    cost1 = total_cost(id_to_vehicle , route_map , vehicleid_to_plan)
    if cost0 - 0.01 < cost1:
        vehicleid_to_plan = backup_restore_solution
    else:
        print(f"After 24h,redispatch valid.originCost:{cost0} newCost:{cost1}improve value: {(cost0 - cost1)}")

def main():
    global before_cost, delta_t , completeOrderItems , newOrderItems , onVehicleOrderItems , unallocatedOrderItems , routeBefore , used_time

    id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_items ,  id_to_ongoing_items , id_to_allorder = Input()
    deal_old_solution_file(id_to_vehicle)
    
    vehicleid_to_plan: Dict[str , List[Node]]= {}
    vehicleid_to_destination : Dict[str , Node] = {}

    new_order_itemIDs : List[str] = []
    new_order_itemIDs = restore_scene_with_single_node(vehicleid_to_plan , id_to_ongoing_items, id_to_unlocated_items  , id_to_vehicle , id_to_factory ,id_to_allorder)

    new_order_itemIDs = [item for item in new_order_itemIDs if item]
    begin_time = time.time()
    
    """Phần cần cải tiến"""
        
    if over24hours(id_to_vehicle):
        redispatch_process(id_to_vehicle , route_map , vehicleid_to_plan , id_to_factory , id_to_unlocated_items)
    else:
        dispatch_new_orders(vehicleid_to_plan , id_to_factory , route_map , id_to_vehicle , id_to_unlocated_items , new_order_itemIDs)
    initial_chromosome : Chromosome = Chromosome(vehicleid_to_plan , route_map , id_to_vehicle)
    
    #GA
    print()
    best_chromosome =  CCEA(initial_chromosome)
    if best_chromosome is None or best_chromosome.fitness > initial_chromosome.fitness:
        best_chromosome = initial_chromosome
    
    print()
    print()
    print(get_route_after(initial_chromosome.solution , {}))
    print(get_route_after(best_chromosome.solution , {}))
    print(f'After GA, the cost of solution decrease from {initial_chromosome.fitness} to {best_chromosome.fitness}')
    
    """" Xử lý lời giải sau tối ưu"""
    used_time = time.time() - begin_time
    update_solution_json(id_to_ongoing_items , id_to_unlocated_items , id_to_vehicle , best_chromosome.solution , vehicleid_to_destination , route_map)
    
    merge_node(id_to_vehicle , best_chromosome.solution)
    
    get_output_solution(id_to_vehicle , best_chromosome.solution , vehicleid_to_destination)
    
    print( f"Destination: {vehicleid_to_destination}", file = sys.stderr)
    write_destination_json_to_file(vehicleid_to_destination  , id_to_vehicle , input_directory)
    
    
    print(f"Route: {best_chromosome.solution}", file = sys.stderr)
    write_route_json_to_file(best_chromosome.solution  , id_to_vehicle , input_directory) 

if __name__ == '__main__':
    main()