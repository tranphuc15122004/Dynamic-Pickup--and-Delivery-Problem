from typing import Dict , List , Tuple 
from algorithm.Object import *
from algorithm.algorithm_config import *
from algorithm.local_search import *
from algorithm.local_search2 import *
import random

def GA(initial_chromosome : Chromosome) -> Chromosome:
    population : List[Chromosome] = []
    PDG_map : Dict[str , List[Node]] = {}
    population ,  PDG_map , base_solution= generate_random_chromosome(initial_chromosome , POPULATION_SIZE)


    if population is None:
        print('Cant initialize the population')
        return None
    
    print()
    print(len(PDG_map))
    print(get_route_after(base_solution , {}))

    best_solution : Chromosome = None
    stagnant_generations = 0  # Biến đếm số thế hệ không cải thiện
    time_of_1_gen = 0
    population.sort(key= lambda x: x.fitness)
    best_solution = population[0]
    begintime = time.time()
    for gen in range(NUMBER_OF_GENERATION):
        new_population = []

        while len(new_population) < POPULATION_SIZE:
            parent1, parent2 = select_parents(population)
            child = parent1.GA_crossover(parent2, PDG_map)
            new_population.append(child)
        # Sắp xếp lại quần thể và lấy 20 cá thể tốt nhất
        population.extend(new_population)
        population.sort(key=lambda x: x.fitness)
        population = population[:POPULATION_SIZE]

        for c in population:
            if random.uniform(0 , 1) <= MUTATION_RATE:
                c.GA_mutate(True)

        # Sắp xếp lại quần thể sau đột biến
        population.sort(key=lambda x: x.fitness)

        # Cập nhật giải pháp tốt nhất từ quần thể đã đột biến
        if best_solution is None or population[0].fitness < best_solution.fitness:
            best_solution = population[0]
            stagnant_generations = 0
        else:
            stagnant_generations += 1

        # Điều kiện dừng sớm nếu không có cải thiện
        if stagnant_generations >= 5:
            print("Stopping early due to lack of improvement.")
            break

        endtime = time.time()
        if time_of_1_gen == 0:
            time_of_1_gen = endtime - begintime
        used_time = endtime - begintime + time_of_1_gen
        if used_time > 10 * 60:
            print("TimeOut!!")
            break
        
        for c in population:
            print(get_route_after(c.solution , {})  , file= sys.stderr)
        print(f'Generation {gen+1}: Best Fitness = {best_solution.fitness}')
    return best_solution    


# Chọn lọc cha mẹ bằng phương pháp tournament selection
def select_parents(population: List[Chromosome]) -> Tuple[Chromosome, Chromosome]:
    def tournament_selection():
        tournament_size = max(2, len(population) // 10)
        candidates = random.sample(population, tournament_size)
        return min(candidates, key=lambda x: x.fitness)
    return tournament_selection(), tournament_selection()

# Tra ve None neu khong the xay dung duoc quan the 
def generate_random_chromosome(initial_chromosome: Chromosome , size: int) -> Tuple[List[Chromosome] ,  Dict[str , List[Node]] , Dict[str , List[Node]]]: 
    dis_order_super_node : Dict[int, Dict[str , Node]] = get_UnongoingSuperNode(initial_chromosome.solution , initial_chromosome.id_to_vehicle)
    ls_node_pair_num = len(dis_order_super_node)
    if ls_node_pair_num == 0:
        return None , None , None

    #Quan the
    population : List[Chromosome] = []
    
    pdg_Map : Dict[str , List[Node]] = {}
    base_solution : Dict[str , List[Node]] = {}
    for vehicleID , plan in initial_chromosome.solution.items():
        base_solution[vehicleID] = []
        for node in plan:
            base_solution[vehicleID].append(node)
    
    # tao Dict cac super node
    del_index = {vehicleID:[] for vehicleID in initial_chromosome.id_to_vehicle.keys()}
    for idx, pdg in dis_order_super_node.items():
        pickup_node = None
        delivery_node = None
        node_list: List[Node] = []
        pos_i = 0
        pos_j = 0
        d_num = len(pdg) // 2
        index = 0

        if pdg:
            vehicleID = ''
            for v_and_pos_str, node in (pdg.items()):
                vehicleID = v_and_pos_str.split(",")[0]
                if index % 2 == 0:
                    pos_i = int(v_and_pos_str.split(",")[1])
                    pickup_node = node
                    node_list.insert(0, pickup_node)
                    index += 1
                    del_index[vehicleID].append(pos_i)
                else:
                    pos_j = int(v_and_pos_str.split(",")[1])
                    delivery_node = node
                    node_list.append(delivery_node)
                    index += 1
                    pos_j = int(pos_j - d_num + 1)
                    del_index[vehicleID].append(pos_j)
            
            k : str = f"{vehicleID},{int(pos_i)}+{int(pos_j)}"
            pdg_Map[k] = node_list
    if len(pdg_Map) < 2:
        return None , None , None

    for vehicleID in del_index:
        del_index[vehicleID].sort(reverse=True)  # Sắp xếp index giảm dần trước khi xóa
        for index in del_index[vehicleID]:
            if 0 <= index < len(base_solution[vehicleID]):  # Kiểm tra index hợp lệ
                del base_solution[vehicleID][index]  # Xóa phần tử ở vị trí index
    
    
    # Tao quan the
    while len(population) < size:
        temp_route: Dict[str , List[Node]] = {}
        for vehicleID , plan in base_solution.items():
            temp_route[vehicleID] = []
            for node in plan:
                temp_route[vehicleID].append(node)
        # Chen ngau nhien cac super node vao cac lo trinh cua cac xe 
        for DPG in pdg_Map.values():
            # Khai bao cac bien lien quan
            is_inserted = False
            # chen theo cách tốt nhất
            if random.uniform(0 , 1) <= 0.25:
                isExhausive = False
                route_node_list : List[Node] = []
                selected_vehicleID = random.choice(list(base_solution.keys()))
                if DPG:
                    isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(DPG , initial_chromosome.id_to_vehicle , temp_route , initial_chromosome.route_map , selected_vehicleID)
                
                route_node_list = temp_route.get(bestInsertVehicleID , [])

                if isExhausive:
                    route_node_list = bestNodeList[:]
                else:
                    if route_node_list is None:
                        route_node_list = []
                    
                    new_order_pickup_node = DPG[0]
                    new_order_delivery_node = DPG[1]
                    
                    route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                    route_node_list.insert(bestInsertPosJ, new_order_delivery_node)
                temp_route[bestInsertVehicleID] = route_node_list
                is_inserted = True
                #print('chèn 1 cặp node vào xe bằng CI')
            else:                
                while (is_inserted == False):
                    selected_vehicleID = random.choice(list(base_solution.keys()))
                    
                    selected_vehicle = initial_chromosome.id_to_vehicle[selected_vehicleID]
                    begin_pos = 1 if selected_vehicle.des else 0
                    check_end = False
                    old_len = len(temp_route[selected_vehicleID])
                    if old_len == 0:
                        temp_route[selected_vehicleID].extend(DPG)
                        is_inserted = True
                    else:
                        pickup_node = DPG[0]
                        delivery_node = DPG[-1]
                        
                        # chen cac cap node vao ngau nhien trong cac xe
                        # chen node nhan truoc                        
                        feasible_position1 = [i for i in range(begin_pos , len(temp_route[selected_vehicleID]) + 1)]
                        random.shuffle(feasible_position1)
                        for insert_posI in feasible_position1:
                            feasible_position2 = [i for i in range(insert_posI +1, len(temp_route[selected_vehicleID]) + 2)]
                            for insert_posJ in feasible_position2:
                                temp_route[selected_vehicleID].insert(insert_posI , pickup_node)
                                temp_route[selected_vehicleID].insert(insert_posJ , delivery_node)
                                
                                if (isFeasible(temp_route[selected_vehicleID] , selected_vehicle.carrying_items ,  selected_vehicle.board_capacity)):
                                    check_end  = True
                                    break
                                
                                temp_route[selected_vehicleID].pop(insert_posJ)
                                temp_route[selected_vehicleID].pop(insert_posI)
                            
                            if check_end:
                                break
                    
                    #kiem tra xem cap node da duojc chen chua
                    if len(temp_route[selected_vehicleID]) == old_len + 2 :
                        is_inserted = True
                
        # Da tao xong mot ca the moi
        if len(temp_route) == len(initial_chromosome.id_to_vehicle):
            population.append(Chromosome(temp_route , initial_chromosome.route_map , initial_chromosome.id_to_vehicle))
    
    return population , pdg_Map , base_solution
