import csv
import json
import string
import sys
from typing import Dict , List , Tuple
from algorithm.Object import *
import pandas as pd
from src.conf.configs import Configs
from src.utils.logging_engine import logger


def read_input_Factory_CSV(file_path : str) -> Dict[str , Factory]:
    #file_path = 'benchmark/factory_info.csv'
    id2factory_map: Dict[str, Factory] = {}
    try:
        df = pd.read_csv(file_path)
        for index, row in df.iterrows():
            factory_id = str(row['factory_id'])
            lng = float(row['longitude'])
            lat = float(row['latitude'])
            dock_num = int(row['port_num'])
            factory = Factory(factory_id, lng, lat, dock_num)
            if factory_id not in id2factory_map:
                id2factory_map[factory_id] = factory
    except Exception as e:
        print(f"Error: {e}", file = sys.stderr)
    return id2factory_map 


def read_input_Routemap_CSV(file_path : str) -> Dict[Tuple[str , str] , Tuple[str , str]]:
    #file_path = 'benchmark/route_info.csv'
    Route_map : Dict[Tuple[str , str] , Tuple[str , str]] = {}
    try:
        route_df = pd.read_csv(file_path)
        Route_map : Dict[Tuple[str , str] , Tuple[str , str]] = {}
        for index, row in route_df.iterrows():
            start_factory_id = str(row['start_factory_id'])
            end_factory_id = str(row['end_factory_id'])
            distance = float(row['distance'])
            transport_time = int(row['time'])
            
            if (start_factory_id , end_factory_id) not in Route_map:
                Route_map[(start_factory_id , end_factory_id)] = (distance , transport_time) 
    except Exception as e:
        print (f"Error: {e}" , file= sys.stderr)
    return Route_map

def read_unlocated_item(file_name: str) -> Dict[str , OrderItem]:
    #path = 'C:\\PHUC\\DPDP\\dpdp_competition\\algorithm\\data_interaction\\unallocated_order_items.json'
    id_to_unlocated_item : Dict[str , OrderItem] = {}
    with open(file_name, 'r') as fd:
        data1111 = fd.read()
        data =  json.loads(data1111)
        for item_json in data:
            id = item_json.get("id")
            type = item_json.get("type")
            order_id = item_json.get("order_id")
            pickup_factory_id = item_json.get("pickup_factory_id")
            delivery_factory_id = item_json.get("delivery_factory_id")
            creation_time = int(item_json.get("creation_time"))
            committed_completion_time = int(item_json.get("committed_completion_time"))
            load_time = int(item_json.get("load_time"))
            unload_time = int(item_json.get("unload_time"))
            delivery_state = int(item_json.get("delivery_state"))
            demand = float(item_json.get("demand"))
            
            temp = OrderItem(id=id , type= type , order_id= order_id , pickup_factory_id= pickup_factory_id , delivery_factory_id= delivery_factory_id , creation_time= creation_time , committed_completion_time= committed_completion_time , load_time= load_time , unload_time= unload_time , delivery_state= delivery_state , demand= demand)
            
            id_to_unlocated_item[id] = temp
    return id_to_unlocated_item
        
def read_ongoing_item(file_path : str) -> Dict[str , OrderItem]:
    #path = 'C:\\PHUC\\DPDP\\dpdp_competition\\algorithm\\data_interaction\\ongoing_order_items.json'
    id_to_ongoing_item = {}
    with open(file_path, 'r') as fd:
        data1111 = fd.read()
        data =  json.loads(data1111)
        for item_json in data:
            id  = item_json.get("id")
            type = item_json.get("type")
            order_id = item_json.get("order_id")
            pickup_factory_id = item_json.get("pickup_factory_id")
            delivery_factory_id = item_json.get("delivery_factory_id")
            creation_time = int(item_json.get("creation_time"))
            committed_completion_time = int(item_json.get("committed_completion_time"))
            load_time = int(item_json.get("load_time"))
            unload_time = int(item_json.get("unload_time"))
            delivery_state = int(item_json.get("delivery_state"))
            demand = float(item_json.get("demand"))
            temp = OrderItem(id=id , type= type , order_id= order_id , pickup_factory_id= pickup_factory_id , delivery_factory_id= delivery_factory_id , creation_time= creation_time , committed_completion_time= committed_completion_time , load_time= load_time , unload_time= unload_time , delivery_state= delivery_state , demand= demand)
            id_to_ongoing_item[id] = temp
    return id_to_ongoing_item

def read_vehicleinfor(path , id_allorder: Dict[str ,OrderItem]) -> Dict[str , Vehicle]:
    #path  = 'C:\\PHUC\\DPDP\\dpdp_competition\\algorithm\\data_interaction\\vehicle_info.json'
    id_to_vehicle = {}
    with open(path , mode= 'r' ) as file:
        data = file.read()
        vehicle_info = json.loads(data)
        for v_json in vehicle_info:
            id = v_json.get("id")
            gps_id = v_json.get("gps_id")
            cur_factory_id = v_json.get("cur_factory_id")
            operation_time = int(v_json.get("operation_time"))
            capacity = float(v_json.get("capacity"))
            update_time = int(v_json.get("update_time"))
            arrive_time_at_current_factory = int(v_json.get("arrive_time_at_current_factory"))
            leave_time_at_current_factory = int(v_json.get("leave_time_at_current_factory"))
            
            carrying_items_json = v_json.get("carrying_items", [])
            carrying_items_list = [item for item in carrying_items_json] 
            carrying_items = []
            for item_id in carrying_items_list:
                if item_id in id_allorder:
                    carrying_items.append(id_allorder[item_id])

            destination_json = v_json.get("destination")
            des = None
            if destination_json is not None:
                factory_id = destination_json.get("factory_id")
                arrive_time = int(destination_json.get("arrive_time"))
                leave_time = int(destination_json.get("leave_time"))

                # Thêm thông tin về các ITEM trả tại điểm đến
                pickup_items_json = destination_json.get("pickup_item_list", [])
                pickup_items_list = [item for item in pickup_items_json]
                pickup_items = []
                for item_id in pickup_items_list:
                    if item_id in id_allorder:
                        pickup_items.append(id_allorder[item_id])

                # Thêm các ITEM nhận tại điểm đến
                delivery_items_json = destination_json.get("delivery_item_list", [])
                delivery_items_list = [item for item in delivery_items_json]
                delivery_items = []
                for item_id in delivery_items_list:
                    if item_id in id_allorder:
                        delivery_items.append(id_allorder[item_id])

                des = Node(factory_id, delivery_items  ,pickup_items , arrive_time, leave_time)
            temp = Vehicle(id=id, gps_id=gps_id, operation_time=operation_time, board_capacity=capacity, carrying_items=carrying_items, des=des)
            temp.set_cur_position_info(cur_factory_id , update_time , arrive_time_at_current_factory , leave_time_at_current_factory)
            id_to_vehicle[id] = temp
    return id_to_vehicle

def Input() ->Tuple[Dict[str, Factory], Dict[Tuple[str , str], Tuple[str , str]] ,Dict[str , Vehicle], Dict[str , OrderItem], Dict[str , OrderItem], Dict[str , OrderItem] ]:
    id_to_factory = read_input_Factory_CSV(Configs.factory_info_file_path)
    route_map =  read_input_Routemap_CSV(Configs.route_info_file_path)

    id_to_unlocated_item = read_unlocated_item(Configs.algorithm_unallocated_order_items_input_path)
    id_to_ongoing_item = read_ongoing_item(Configs.algorithm_ongoing_order_items_input_path)
    id_allorder : Dict[str , OrderItem] = {}
    id_allorder = {**id_to_unlocated_item, **id_to_ongoing_item}

    id_to_vehicle = read_vehicleinfor(Configs.algorithm_vehicle_input_info_path ,id_allorder)
    
    return id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item , id_allorder
