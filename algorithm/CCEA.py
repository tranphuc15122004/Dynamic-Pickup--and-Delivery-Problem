from collections import defaultdict
from typing import Dict , List , Tuple 
from algorithm.Object import *
from algorithm.algorithm_config import *
from algorithm.local_search import *
from algorithm.local_search2 import *
from algorithm.GA import generate_random_chromosome 
import random
from scipy.spatial.distance import euclidean


def CCEA( initial_chromosome : Chromosome) -> Chromosome:
    population1 : List[Chromosome] = []
    population2 : List[Chromosome] = []
    
    PDG_map : Dict[str , List[Node]] = {}
    population1 ,  PDG_map , base_solution= generate_random_chromosome(initial_chromosome ,2 *  POPULATION_SIZE)

    if (population1 is None) :
        print('Cant initialize the population')
        return None
    
    population1.append(initial_chromosome)
    begin_time = time.time()
    best_solution : Chromosome = None
    stagnant_generations = 0  # Biến đếm số thế hệ không cải thiện
    time_of_1_gen = 0
    population1.sort(key= lambda x: x.fitness)
    best_solution = population1[0]
    
    # Giai doan dau tien 
    # Tiến hóa với mục tiêu về thời gian
    for gen in range (NUMBER_OF_GENERATION):
        offspring :  List[Chromosome] = []
        while (len(offspring) < 2 * POPULATION_SIZE):
            parent1 , parent2 = select_parents(population1)
            child = parent1.CCEA_crossover(parent2 , PDG_map)
            offspring.append(child)
        
        population1.extend(offspring)
        population1.sort(key= lambda x: x.factorial_cost[1])
                
        for c in population1:
            c.random_mutate_operator(True , mode='overtime')
        population1.sort(key= lambda x: x.factorial_cost[1])
        
        
        # Cập nhật giải pháp tốt nhất từ quần thể đã đột biến
        if best_solution is None or population1[0].fitness < best_solution.fitness:
            best_solution = population1[0]
            stagnant_generations = 0
        else:
            stagnant_generations += 1

        # Điều kiện dừng sớm nếu không có cải thiện
        if stagnant_generations >= STAGNANT_THRESHOLD:
            print("Stopping early due to lack of improvement in phase 1.")
            break
        
        endtime = time.time()
        if time_of_1_gen == 0:
            time_of_1_gen = endtime - begin_time
        used_time = endtime - begin_time + time_of_1_gen
        if used_time > 5 * 60:
            print("TimeOut!!")
            break
        
        print(f'Generation {gen+1}: Best Fitness = {best_solution.factorial_cost[1]} , Best individual in the population = {population1[0].factorial_cost}')
    
    population1 = diversity_filter(population1)    
    population1.sort(key= lambda x: x.factorial_cost[1])
    
    for c in population1:
        population2.append(c)
    while len(population2) < POPULATION_SIZE:
        parent1 , parent2 = select_parents(population2)
        child = parent1.CCEA_crossover(parent2 , PDG_map)
        population2.append(child)
    
    population2.sort(key= lambda x: x.fitness)
    best_solution = population2[0]
    stagnant_generations = 0  # Biến đếm số thế hệ không cải thiện
    time_of_1_gen = 0
    
    # Giai đoạn thứ 2
    # Tiến hóa không mất mát với mục tiêu thứ 1 , sử dụng sắp xếp không trội
    for gen in range(NUMBER_OF_GENERATION):
        offspring : List[Chromosome] = []
        while (len(offspring) < NUMBER_OF_GENERATION):
            parent1 , parent2 = select_parents(population2)
            child = parent1.CCEA_crossover(parent2 , PDG_map)
            offspring.append(child)
        
        population2.extend(offspring)
        
        # Sắp xếp không trội
        fronts = non_dominated_sort(population2)

        # Chọn POPULATION_SIZE cá thể tốt nhất
        new_population = []
        for front in fronts:
            if len(new_population) + len(front) <= POPULATION_SIZE:
                new_population.extend(front)
            else:
                # Sắp xếp front cuối cùng theo crowding distance
                front.sort(key=lambda x: calculate_crowding_distance(x, front), reverse=True)
                new_population.extend(front[:POPULATION_SIZE - len(new_population)])
                break

        population2 = new_population
        population2.sort(key= lambda x: x.fitness)
        
        # Dot bien
        for c in population2:
            c.CCEA_mutate(True)

        if best_solution is None or (population2[0].fitness < best_solution.fitness):
            best_solution = population2[0]
            stagnant_generations = 0
        else:
            stagnant_generations += 1
            
        # Điều kiện dừng sớm nếu không có cải thiện
        if stagnant_generations >= STAGNANT_THRESHOLD:
            print("Stopping early due to lack of improvement in phase 2.")
            break
        
        endtime = time.time()
        if time_of_1_gen == 0:
            time_of_1_gen = endtime - begin_time
        used_time = endtime - begin_time + time_of_1_gen
        if used_time > 10 * 60:
            print("TimeOut!!")
            break
        
        print(f'Generation {gen+1}: Best Fitness = {best_solution.fitness} , Best individual in the population = {population2[0].factorial_cost}')
    
    return best_solution


def select_parents(population: List[Chromosome]) -> Tuple[Chromosome, Chromosome]:
    def tournament_selection():
        tournament_size = max(2, len(population) // 10)
        candidates = random.sample(population, tournament_size)
        return min(candidates, key=lambda x: x.fitness)
    return tournament_selection(), tournament_selection()


def non_dominated_sort(population: List[Chromosome]) -> List[List[Chromosome]]:
    """
    Thực hiện sắp xếp không trội (non-dominated sorting).
    Input: population - danh sách các cá thể (Chromosome).
    Output: danh sách các fronts, mỗi front là một danh sách các cá thể không bị chi phối lẫn nhau.
    """
    if not population:
        return []

    # Khởi tạo các biến
    domination_count = {id(c): 0 for c in population}  # Số cá thể chi phối c
    dominated_solutions = {id(c): [] for c in population}  # Danh sách các cá thể bị c chi phối
    fronts = [[]]  # Danh sách các fronts, fronts[0] là Front 1

    # So sánh từng cặp cá thể để xác định chi phối
    for i, p in enumerate(population):
        for q in population[i + 1:]:
            p_cost = p.factorial_cost
            q_cost = q.factorial_cost
            
            # Kiểm tra p chi phối q
            p_dominates_q = all(p_c <= q_c for p_c, q_c in zip(p_cost, q_cost)) and any(
                p_c < q_c for p_c, q_c in zip(p_cost, q_cost)
            )
            # Kiểm tra q chi phối p
            q_dominates_p = all(q_c <= p_c for p_c, q_c in zip(p_cost, q_cost)) and any(
                q_c < p_c for p_c, q_c in zip(p_cost, q_cost)
            )

            if p_dominates_q:
                dominated_solutions[id(p)].append(q)
                domination_count[id(q)] += 1
            if q_dominates_p:
                dominated_solutions[id(q)].append(p)
                domination_count[id(p)] += 1

    # Xác định Front 1: các cá thể không bị ai chi phối
    for c in population:
        if domination_count[id(c)] == 0:
            fronts[0].append(c)

    # Xác định các front tiếp theo
    i = 0
    while fronts[i]:
        next_front = []
        for p in fronts[i]:
            for q in dominated_solutions[id(p)]:
                domination_count[id(q)] -= 1
                if domination_count[id(q)] == 0:
                    next_front.append(q)
        fronts.append(next_front)
        i += 1

    # Loại bỏ front rỗng cuối cùng
    return fronts[:-1]

from typing import List
from scipy.spatial.distance import euclidean

""" def calculate_crowding_distance(chromosome: Chromosome, front: List[Chromosome]) -> float:
    if len(front) <= 2:
        return float('inf')

    # Khởi tạo khoảng cách đông đúc
    distances = {id(c): 0.0 for c in front}
    num_objectives = len(chromosome.factorial_cost)

    # Chuẩn hóa factorial_cost cho mỗi mục tiêu
    normalized_costs = [[] for _ in range(num_objectives)]  # Lưu giá trị chuẩn hóa
    min_vals = [float('inf')] * num_objectives
    max_vals = [float('-inf')] * num_objectives

    # Tìm min và max cho mỗi mục tiêu
    for c in front:
        for m in range(num_objectives):
            cost = c.factorial_cost[m]
            min_vals[m] = min(min_vals[m], cost)
            max_vals[m] = max(max_vals[m], cost)

    # Chuẩn hóa Min-Max
    for c in front:
        for m in range(num_objectives):
            if max_vals[m] == min_vals[m]:  # Tránh chia cho 0
                normalized_cost = 0.0
            else:
                normalized_cost = (c.factorial_cost[m] - min_vals[m]) / (max_vals[m] - min_vals[m])
            normalized_costs[m].append((normalized_cost, c))

    # Tính khoảng cách đông đúc dựa trên giá trị chuẩn hóa
    for m in range(num_objectives):
        # Sắp xếp front theo giá trị chuẩn hóa của mục tiêu m
        sorted_front = sorted(normalized_costs[m], key=lambda x: x[0])
        
        # Gán khoảng cách vô cực cho cá thể ở biên (đầu và cuối)
        distances[id(sorted_front[0][1])] = float('inf')
        distances[id(sorted_front[-1][1])] = float('inf')

        # Tính khoảng cách cho các cá thể ở giữa
        for i in range(1, len(sorted_front) - 1):
            current_chrom = sorted_front[i][1]
            # Khoảng cách dựa trên giá trị chuẩn hóa (đã nằm trong [0, 1])
            distance_contribution = sorted_front[i + 1][0] - sorted_front[i - 1][0]
            distances[id(current_chrom)] += distance_contribution

    return distances[id(chromosome)] """

def calculate_crowding_distance(chromosome: Chromosome, front: List[Chromosome]) -> float:
    if len(front) <= 2:
        return float('inf')

    distances = {id(c): 0.0 for c in front}
    for m in range(len(chromosome.factorial_cost)):
        sorted_front = sorted(front, key=lambda x: x.factorial_cost[m])
        min_val = sorted_front[0].factorial_cost[m]
        max_val = sorted_front[-1].factorial_cost[m]
        range_val = max_val - min_val if max_val != min_val else 1.0

        distances[id(sorted_front[0])] = float('inf')
        distances[id(sorted_front[-1])] = float('inf')  

        for i in range(1, len(sorted_front) - 1):
            distances[id(sorted_front[i])] += (
                (sorted_front[i + 1].factorial_cost[m] - sorted_front[i - 1].factorial_cost[m]) / range_val
            )

    return distances[id(chromosome)] 

def calculate_distance(c1: Chromosome, c2: Chromosome) -> float:
    return euclidean(c1.factorial_cost, c2.factorial_cost)

def diversity_filter(population: List[Chromosome]) -> List[Chromosome]:
    population.sort(key= lambda x: x.factorial_cost[1])
    filtered = population[:POPULATION_SIZE]
    
    temp_count : Dict[float , List[Chromosome]] = {}
    for c in filtered:
        key = c.factorial_cost[1]
        if key not in temp_count.keys():
            temp_count[key] =  []
        temp_count[c.factorial_cost[1]].append(c)
    for value in temp_count.values():
        value.sort(key= lambda x: x.fitness)
    
    result = []
    for value in temp_count.values():
        if len(value) >= 3:
            result.extend(value[:3])
        else:
            result.extend(value)
    return result