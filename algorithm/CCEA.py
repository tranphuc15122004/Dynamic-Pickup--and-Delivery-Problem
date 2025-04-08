from typing import Dict , List , Tuple 
from algorithm.Object import *
from algorithm.algorithm_config import *
from algorithm.local_search import *
from algorithm.local_search2 import *
from algorithm.GA import generate_random_chromosome 
import random


def CCEA( initial_chromosome : Chromosome) -> Chromosome:
    population : List[Chromosome] = []
    PDG_map : Dict[str , List[Node]] = {}
    population ,  PDG_map , base_solution= generate_random_chromosome(initial_chromosome ,2 *  POPULATION_SIZE)

    if (population is None) :
        print('Cant initialize the population')
        return None
    
    population_1 = population[:POPULATION_SIZE]
    population_2 = population[POPULATION_SIZE:]
    
    print()
    print(len(PDG_map))
    print(get_route_after(base_solution , {}))
    
    population_1.append(initial_chromosome)
    population_2.append(initial_chromosome)
    
    
    stagnant_generations : List[int] = [0 , 0]  # Biến đếm số thế hệ không cải thiện
    time_of_1_gen = 0
    begintime = time.time()
    best_solution : List[Chromosome] = [None , None]
    
    for gen in range(NUMBER_OF_GENERATION):
        for idx , P in enumerate([population_1 , population_2]):
            # Tiến hóa một quần thể
            new_population = []
            mode = 'overtime' if idx == 1 else 'total'
            
            while len(new_population) < POPULATION_SIZE:
                parent1, parent2 = select_parents(P , idx)
                child = parent1.CCEA_crossover(parent2, PDG_map, mode)
                new_population.append(child)
                
            P.extend(new_population)
            P.sort(key=lambda x: (x.factorial_cost[idx], x.factorial_cost[1 - idx]))
            del P[POPULATION_SIZE:]
            
            for c in P:
                if random.uniform(0 , 1) <= MUTATION_RATE:
                    c.CCEA_mutate(True , mode )
            
            P.sort(key=lambda x: (x.factorial_cost[idx], x.factorial_cost[1 - idx]))
            # Cập nhật giải pháp tốt nhất từ quần thể đã đột biến
            if best_solution[idx] is None or (P[0].factorial_cost[idx] < best_solution[idx].factorial_cost[idx]):
                best_solution[idx] = P[0]
                stagnant_generations[idx] = 0
            else:
                stagnant_generations[idx] += 1

        # Điều kiện dừng sớm nếu không có cải thiện
        if stagnant_generations[0] >= 5 and stagnant_generations[-1] >= 5:
            print("Stopping early due to lack of improvement.")
            break
        
        population_1 , population_2 = migaration(population_1 , population_2)
        
        endtime = time.time()
        if time_of_1_gen == 0:
            time_of_1_gen = endtime - begintime
        used_time = endtime - begintime + time_of_1_gen
        if used_time > 10 * 60:
            print("TimeOut!!")
            break
        
        print(f'Generation {gen+1}: \nBest Fitness of population 1 = {best_solution[0].factorial_cost} , the worst = {population_1[-1].factorial_cost} , number of solution {len(population_1)}')
        print(f'Best Fitness of population 2 = {best_solution[1].factorial_cost} , the worst = {population_2[-1].factorial_cost} , number of solution {len(population_2)}')
    
    for mode in ['overtime' , 'distance' , 'total']:
        temp = best_solution[0].CCEA_crossover(best_solution[1] , PDG_map , mode)
        best_solution.append(temp)
    best_solution.sort(key=lambda x: x.fitness)
    return best_solution[0]       

def select_parents(population: List[Chromosome] , idx) -> Tuple[Chromosome, Chromosome]:
    def tournament_selection():
        tournament_size = max(2, len(population) // 10)
        candidates = random.sample(population, tournament_size)
        return min(candidates, key=lambda x: (x.factorial_cost[idx], x.factorial_cost[1 - idx]))
    return tournament_selection(), tournament_selection()

def migaration(population1 : List[Chromosome] , population2: List[Chromosome])-> Tuple[List[Chromosome] , List[Chromosome]]:
    population1.sort(key=lambda x: x.factorial_cost[1])
    population2.sort(key=lambda x: x.factorial_cost[0])
    
    temp_population1 = population1[:POPULATION_SIZE // 4]
    temp_population2 = population2[:POPULATION_SIZE // 4]
    
    new_population1 = []
    for c in temp_population2:    
        new_population1.append(c)
    new_population1.extend(population1)
    
    new_population2 = []
    for c in temp_population1:    
        new_population2.append(c)
    new_population2.extend(population2)
    
    return new_population1 , new_population2