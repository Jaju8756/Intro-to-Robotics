# Write a Python function sequence that takes one input, namely the maximum sum Sm.
# This function must compute the minimum number of terms such that the sum of terms
# is greater than Sm. These terms must be returned in an array (or list). Example: The
# input 22 should return a list: [2, 5, 10, 17].
def sequence(Sm):
    current_sum = 0 #initializing all the local variables used
    terms = []
    term = 1

    while(current_sum <=Sm): #sum of the terms must be less than Sm
        new_term = (term * term) +1 # each new_term can be found using the formula k^2 +1
        terms.append(new_term) # appending the terms in the list
        term +=1
        current_sum += new_term # current_sum updated with the latest sum in the list
    return terms

input_Sm = int(input())
SumList = sequence(input_Sm)
print(SumList)


# Write a Python function smallest multiple that takes a list of positive whole
# numbers as input. This function must compute and return the smallest whole number
# that can be divided by each of the numbers in the input array without a remainder.
# Example: The input [2, 4, 7] should return 28.
import math

def lcm(a, b):
    return abs(a * b) // math.gcd(a, b) # // operator to flooring the result by discarding the fractional part of the result gcd

def smallest_multiple(Pos: list): # basically to find the LCM
    temp = Pos[0]
    for num in Pos[1:]: # loops starting from the second element in the list till the last element
        temp = lcm(temp, num)
    return temp

listNum = [4, 3, 5, 7]
smallest_whole = smallest_multiple(listNum)
print(smallest_whole)


# Write a Python program that reproduces the figure below with all annotations included (axis
# labels, title, tick marks, etc.). The Figure is produced by plotting f (a) as a function of the
# upper bound a of the integral. The upper bound a is discretized between -2 and 2 using
# increments of 0.01. The function quad in the scipy.integrate subpackage may be useful for
# this problem

import numpy as np
from matplotlib import pyplot as plt
from scipy.integrate import quad

def calculate(x):
    return x**2 / np.sqrt(1+x+x**2) # formula for integration

def f(a):
    result, error = quad(calculate, -2, a) # quad returns two values: result of integration and error, It's important to assign it to two variables as only the result is required here.
    return result

a_values = np.arange(-2.0, 2.0, 0.01) #specifying the range from -2 to 2 in steps of 0.01
f_values = [f(a) for a in a_values[:]] # calculating f for all values of a

plt.plot(a_values, f_values)
plt.title("The integral f(a) for various values of a")
plt.xlabel("Upper bound, a")
plt.ylabel("Integral value, f(a)")
plt.grid(True) # to get the graph with grids
plt.legend("f(a)")
plt.show()


# A circular prime number is a prime number of which all permutations of its digits
# are also prime. For example, 199 is a circular prime number as all permutations of digits
# [199]: 199, 919, 991 are also prime numbers.
# Write a function cprimes list which has one argument, namely the integer N . The function
# must compute and return a numpy array of the first N circular prime numbers. The array of
# circular prime numbers must contain unique entries in ascending order.
# For example, the call cprimes list(9) must return the first 9 circular prime numbers in a
# numpy array: [2, 3, 5, 7, 11, 13, 17, 31, 37]. Hints: The function permutations in the itertools
# module may be useful for this problem. Also consider using a ”set” data structure to remove
# repeated numbers.

import itertools
import numpy as np

def is_prime(pNum):
    if pNum < 2:   #no number less than 2 is prime
        return False
    for i in range(2, int(pNum**0.5)+1):  #prime number check
        if pNum % i ==0:
            return False
    return True

def is_circular_prime(cPrime):
    cPrimeStr = str(cPrime)
    permuts = set(int(''.join(p)) for p in itertools.permutations(cPrimeStr)) #find all the possible permutations of any number by passing the number as string
    return all(is_prime(p) for p in permuts) # returns true only if all the elements in permuts are prime
                            # for num in permuts: 
                            #     if is_prime(num) == True:
                            #         return True
                            #     return False

def cprimes_list(N): 
    numpy_array = set() # doesn't allow duplicates of elements
    number = 2 # prime number starts from 2
    i=0
    while len(numpy_array) < N: # finding out first N cprime numbers
        if is_circular_prime(number):
            numpy_array.add(number) # storing the numbers in an array 
        number+=1
    return np.array(sorted(numpy_array)) # get the array sorted

list = cprimes_list(11)
print(list)


# Linear regression helps us find a mathematical model from data in the form of
# y = mx + b where x represents the independent variable (predictor), y represents the depen-
# dent variable (target) and m and b are model parameters determined through the regression
# process. Multiple linear regression is an extension of linear regression where multiple inde-
# pendent variables are present creating a model of the form z = ax + by + c. Notice that there
# are now two independent variables, x and y and a single dependent variable z.
# For this problem, import data from the file q5.csv to determine a multiple linear regression
# model fitting the x and y independent variables to the z dependent variable (the data columns
# in the file are ordered x, y, z). Your code should print out the relevant coefficients in the
# order [a, b, c]. You are also required to generate a 3-D scatter plot similar to the one shown
# in the figure below. Hint: The module LinearRegression from sklearn.linear model may be
# useful for this problem. Use the loadtext function from Numpy to load the .csv file.

import numpy as np
from sklearn.linear_model import LinearRegression
from matplotlib import pyplot as plt

data = np.loadtxt('q5.csv', delimiter=',') # loading the csv file which has the data separted by comma

XY = data[:, 0:2] # data is obtained by assigning all the data from the first 2 rows to XY
Z = data[:, 2] # estimate is obtained by asigning all the data from the third row

model = LinearRegression() # model represents the linear regression equation 
model.fit(XY, Z) # fitting XY and Z to model

a, b = model.coef_  # Coefficients for x and y
c = model.intercept_  # Intercept
print(f"a={a}, b={b}, c={c}")
pred = model.predict(XY) #predicting the model

fig = plt.figure()
td = fig.add_subplot(111, projection='3d') #ploting a 3d model

td.scatter(XY[:, 0], XY[:, 1], Z, c = 'red', marker='o', label='Data') # scattering the estimated model
td.scatter(XY[:, 0], XY[:, 1], pred, c= 'blue', marker='o', label='Estimates')# scattering the predicted model
td.legend()

td.set_xlabel('X')
td.set_ylabel('Y')
td.set_zlabel('Z')

plt.show()


# Define a class named Square which inherits the Rectangle class. The sides of a
# square are all of equal length, which makes the square a special case of the rectangle. The
# new class should not contain any new attributes. See file q6.py file for skeleton code. The
# client code in the .py file is expected to produce the output:
# square 4x4
# area: 16

class Rectangle:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    def __str__(self):
        return f"rectangle {self.width}x{self.height}"
    
    def area(self):
        return self.width * self.height
    
class Square(Rectangle):
    def __init__(self, width):
        super().__init__(width, width) # calling the base class constructor

    def __str__(self):
        return f"square {self.width}x{self.width}" # overidding __str__ in base class
    
square = Square(4) #calls the derived class constructor -> width =4
print(square) #invokes the __str__ of Square
print("area:", square.area()) #invokes the base class function . area() is public and inherited to Square 


# The file q7.py contains code for a two-player word game. The parent class
# WordGames contains attributes that keeps track of how many games each player has won
# and also the number of rounds a particular game will last. The method round winner de-
# termines which player won the previously played round based on their input to the console.
# This input is generated from the play function that handles the gameplay for each round.
# The nominal round winner method determines the round winner by chance.
# Define a class RockPaperScissors which allows you to play a game of rock-paper-scissors.
# The rules are as follows:
# • rock beats scissors (the rock can break the scissors but the scissors can’t cut the rock)
# • paper beats rock (the paper can cover the rock)
# • scissors beats paper (the scissors can cut the paper)
# The class RockPaperScissors is required to inherit from the WordGame class already
# defined and will override the round winner method. If the input from either player is invalid,
# they lose the round. If both players type in something other than rock, paper or scissors, the
# result for that round is a tie.

import random

class WordGame():
    def __init__(self, rounds: int):
        self.wins1 = 0
        self.wins2 = 0
        self.rounds = rounds
    def round_winner(self, player1_word: str, player2_word: str): # WordGame uses this function to return values
        # determine a random winner
        return random.randint(1, 2)
    def play(self):
        print("Word game:")
        for i in range(1, self.rounds+1):
            print(f"round {i}")
            answer1 = input("player1: ")
            answer2 = input("player2: ")
            if self.round_winner(answer1, answer2) == 1:
                self.wins1 += 1
                print("player 1 won")
            elif self.round_winner(answer1, answer2) == 2:
                self.wins2 += 1
                print("player 2 won")
            else:
                print("It's a tie")
        print("game over, wins:")
        print(f"player 1: {self.wins1}")
        print(f"player 2: {self.wins2}")
class RockPaperScissors(WordGame):
    def __init__(self, rounds: int):
        super().__init__(rounds)

    def round_winner(self, player1_word: str, player2_word: str): # class RockPaperScissors calls this function to return value 
        set = ['rock', 'paper', 'scissors'] # ensuring the elements are unique
        if player1_word not in set and player2_word not in set: #checking corner cases
            return 0  
        elif player1_word not in set:
            return 2 
        elif player2_word not in set:
            return 1 
        
        if player1_word == player2_word:
            return 0  
        
        if (player1_word == 'rock' and player2_word == 'scissors') or \
           (player1_word == 'scissors' and player2_word == 'paper') or \
           (player1_word == 'paper' and player2_word == 'rock'):
            return 1  
        else:
            return 2  
        
game = RockPaperScissors(rounds=3) 
game.play()
