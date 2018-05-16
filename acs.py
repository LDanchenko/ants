from random import shuffle, uniform
from sys import float_info

import numpy as np
from geopy.geocoders import Nominatim
from geopy import distance

def euc_2d(c1, c2):
    """
    Рассчитывает Эвклидово расстояние в двухмерном пространстве между
    точками `c1` и `c2`.

    :param c1: Координаты точки начала.
    :param c2: Координаты точки конца.
    :return: Эвклидово растояние между двумя точками(`c1`, `c2`).
    """
    point1 =   (c1[0],c1[1])
    point2 =   (c2[0],c2[1])
    return (distance.distance(point1, point2).km)

# return distance.distance(c1, c2).km

def initialize_ph(cities, ph):
    """
    Инициализация матрицы феромонов

    :param cities: Список городов
    :param ph: Матрица феромонов - двумерный список
    """
    way = cities[:]
    shuffle(way)

    for i in range(len(way)):
        for j in range(len(way)):
            ph[i][j] = 1 / fitness(way)


def fitness(way):
    """
    Расчет оценки(score) для даного маршрута.

    :param way: Маршрут, для которого будет проводится оценка.
    :return: score маршрута.
    """
    value = 0
    for i in range(len(way) - 1):
        value += euc_2d(way[1], way[i + 1])

    return value

def update_pg_global(alpha, way, ph):
    cost = fitness(way)
    for i in range(len(ph)):
        for j in range(len(ph)):
            ph[i][j] = ((1-alpha)* ph[i][j]) + alpha * (1 / cost)
    print(ph)

def update_ph(ph, cities, way, p):
    """
    Обновление матрицы феромонов

    :param ph: Матрица феромонов
    :param cities: Список городов
    :param way: Маршрут
    """
    cost = fitness(way)
    if cost == 0:
        cost = float_info.epsilon
    for i in range(len(way) - 1): #????
        c_from = cities.index(way[i])
        c_to = cities.index(way[i + 1])
        ph[c_from][c_to] = (1 - p) * (ph[c_from][c_to]) + p * ( 1 / cost)


def decay_ph(ph, alpha):
    """
    Испарение феромона (на всей матрице)

    :param ph: Матрица феромонов
    :param alpha: Значение на которое изменяются феромоны
    """
    for i in range(len(ph)):
        for j in range(len(ph)):
            ph[i][j] **= 1 - alpha


def STR(ph, beta, cities, way, q0):
    """
    state transition rule. Возвращает следующий наиболее подходящий
    город при заданных значениях с использованиям "правила рулетки".

    :param ph: Матрица феромонов
    :param beta: Степень влияния феромонов.
    :param cities: Список городов
    :param way: Пройденный маршрут
    :return: Кортеж, с координатами города.
    """
    #q0-задает пользователь
    transitions = set(cities) - set(way)
    c_from = cities.index(way[-1])
    q = uniform(0,1)
    s = []
    transitions_l=list(transitions)
    for item in transitions:
        s.append(ph[c_from][cities.index(item)] * (1 / euc_2d(way[-1], item)) ** beta)

    # правило рулетки
    if (q<=q0):
        return transitions_l[s.index(max(s))]
    else:
        probability = uniform(0, 1)
        total_pr = 0
        for item in transitions:
            total_pr += (ph[c_from][cities.index(item)] * (1 / euc_2d(way[-1], item)) ** beta) / sum(s)
            if probability <= total_pr:
                return item


def search(cities, alpha, betta, n, m, q0, first_pos=0 ):
    """
    Поиск маршрута, проходящего через задание города(`cities`).

    :param cities: Список городов.
    :param alpha: Зн. на которое изменяются феромоны на итерации.
    :param betta: Степень влияния феромонов..
    :param m: Колличество муравьем
    :param n: колличество муравейиков (колоний)
    :param first_pos: Индекс города с которого начинаем маршрут
    :return: Список городов в порядке посещения.
    """
    ph = [[0] * len(cities) for _ in range(len(cities))]
    initialize_ph(cities, ph)

    best_way = None
    best_cost = 0
    for j in range(n):
        for k in range(m):
            current, way = first_pos, []
            way.append(cities[current])

            while len(way) < len(cities):
                current = STR(ph, betta, cities, way, q0)
                way.append(current)
         #       decay_ph(ph, alpha)
                cost = fitness(way)
                update_ph(ph, cities, way, 0.1)

                if best_cost == 0 or cost < best_cost:
                     best_cost = cost
                     best_way = way
        #decay_ph(ph,alpha)
        update_pg_global(alpha,best_way,ph)
    return best_way



def demonstrate():
    """
    Демонстрация примера использования, с виводом на стандартный поток.
    """
    geolocator = Nominatim()
    f = open("cities.txt") #открыли файл
    cities = []
    #инициализируем массив координатами городов
    #for line in f.readlines():
     #   location = geolocator.geocode(line)
      #  cities.append((location.latitude, location.longitude))  # широта долгот

    cities = [(0, 20), (20, 50), (30, 20), (50, 100),(67, 56)]
    alpha = 5
    q0= 0.5
    betta = 1
    way = search(cities, alpha, betta, 2, 20, q0)
    print(fitness(way))
    print(way)
    f.close()

demonstrate()
