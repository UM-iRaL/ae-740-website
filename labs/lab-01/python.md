---
layout: default
title: "Python"
parent: "Lab 1"
grand_parent: Labs
nav_order: 4
has_toc: true
---

# Python
{: .no_toc}

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Python Virtual Environments (venv)

Virtual environments are isolated Python environments that allow you to install packages for specific projects without affecting your system Python installation.

### Why Use Virtual Environments?

- **Isolation**: Keep project dependencies separate
- **Version control**: Use different versions of packages for different projects
- **Clean system**: Avoid cluttering your system Python installation
- **Reproducibility**: Share exact package versions with others

### Creating and Using Virtual Environments

```bash
# Create a new virtual environment
python3 -m venv ae740_venv

# Activate the virtual environment
source ae740_venv/bin/activate

# Your prompt should change to show the environment name
(ae740_venv) user@computer:~$
```

### Managing Packages in Virtual Environment

```bash
# Install packages (only affects current environment)
pip install numpy
pip install matplotlib scipy pandas

# Install specific versions
pip install numpy==1.21.0

# Install from requirements file
pip install -r requirements.txt

# List installed packages
pip list

# Create requirements file
pip freeze > requirements.txt
```

### Example requirements.txt

```text
numpy==1.24.3
matplotlib==3.7.1
scipy==1.10.1
pandas==2.0.2
jupyter==1.0.0
```

### Deactivating and Removing Environments

```bash
# Deactivate current environment
deactivate

# Remove virtual environment (just delete the folder)
rm -rf ae740_venv
```



## Python Basics

### Variables and Data Types

```python
# Numbers
integer_num = 42
float_num = 3.14
complex_num = 2 + 3j

# Strings
text = "Hello, Python!"
multiline = """This is a
multiline string"""

# Booleans
is_true = True
is_false = False

# Lists
numbers = [1, 2, 3, 4, 5]
mixed_list = [1, "hello", 3.14, True]

# Dictionaries
person = {"name": "Alice", "age": 30, "city": "Ann Arbor"}
```

### Control Flow

```python
# If statements
x = 10
if x > 5:
    print("x is greater than 5")
elif x == 5:
    print("x equals 5")
else:
    print("x is less than 5")

# For loops
for i in range(5):
    print(f"Iteration {i}")

# While loops
count = 0
while count < 3:
    print(f"Count: {count}")
    count += 1

# List comprehensions
squares = [x**2 for x in range(10)]
even_squares = [x**2 for x in range(10) if x % 2 == 0]
```

### Functions

```python
def greet(name, age=None):
    """Function to greet a person."""
    if age:
        return f"Hello {name}, you are {age} years old!"
    else:
        return f"Hello {name}!"

# Function call
message = greet("Alice", 25)
print(message)

# Lambda functions
square = lambda x: x**2
print(square(5))  # Output: 25
```

## NumPy Tutorial

### Installation and Import

```python
# Install NumPy (run in terminal)
# pip install numpy

import numpy as np
```

### Creating Arrays

```python
# From lists
arr1 = np.array([1, 2, 3, 4, 5])
arr2d = np.array([[1, 2, 3], [4, 5, 6]])

# Built-in functions
zeros = np.zeros((3, 3))
ones = np.ones((2, 4))
eye = np.eye(3)  # Identity matrix
range_arr = np.arange(0, 10, 2)  # [0, 2, 4, 6, 8]
linspace = np.linspace(0, 1, 5)  # [0, 0.25, 0.5, 0.75, 1]

# Random arrays
random_arr = np.random.random((3, 3))
normal_arr = np.random.normal(0, 1, (2, 2))
```

### Array Properties

```python
arr = np.array([[1, 2, 3], [4, 5, 6]])

print(f"Shape: {arr.shape}")      # (2, 3)
print(f"Size: {arr.size}")        # 6
print(f"Dimensions: {arr.ndim}")  # 2
print(f"Data type: {arr.dtype}")  # int64
```

### Array Operations

```python
a = np.array([1, 2, 3, 4])
b = np.array([5, 6, 7, 8])

# Element-wise operations
addition = a + b      # [6, 8, 10, 12]
subtraction = a - b   # [-4, -4, -4, -4]
multiplication = a * b  # [5, 12, 21, 32]
division = a / b      # [0.2, 0.33, 0.43, 0.5]

# Mathematical functions
sqrt_a = np.sqrt(a)
sin_a = np.sin(a)
exp_a = np.exp(a)
```

### Matrix Operations

```python
# 2D arrays (matrices)
A = np.array([[1, 2], [3, 4]])
B = np.array([[5, 6], [7, 8]])

# Matrix multiplication
matrix_mult = np.dot(A, B)  # or A @ B
# Result: [[19, 22], [43, 50]]

# Transpose
A_T = A.T  # or np.transpose(A)

# Determinant and inverse
det_A = np.linalg.det(A)
inv_A = np.linalg.inv(A)

# Eigenvalues and eigenvectors
eigenvals, eigenvecs = np.linalg.eig(A)
```

### Array Indexing and Slicing

```python
arr = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

# Basic indexing
print(arr[0])    # 0
print(arr[-1])   # 9

# Slicing
print(arr[2:5])  # [2, 3, 4]
print(arr[::2])  # [0, 2, 4, 6, 8]

# 2D array indexing
arr2d = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
print(arr2d[1, 2])    # 6
print(arr2d[0:2, 1:])  # [[2, 3], [5, 6]]

# Boolean indexing
mask = arr > 5
print(arr[mask])  # [6, 7, 8, 9]
```

### Reshaping and Concatenation

```python
arr = np.arange(12)

# Reshaping
reshaped = arr.reshape(3, 4)
flattened = reshaped.flatten()

# Concatenation
a = np.array([1, 2, 3])
b = np.array([4, 5, 6])
concatenated = np.concatenate([a, b])  # [1, 2, 3, 4, 5, 6]

# Stacking
stacked_v = np.vstack([a, b])  # Vertical stack
stacked_h = np.hstack([a, b])  # Horizontal stack
```

### Statistical Operations

```python
data = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

# Basic statistics
mean = np.mean(data)        # 5.0
median = np.median(data)    # 5.0
std = np.std(data)          # 2.58
var = np.var(data)          # 6.67

# Along specific axes
mean_axis0 = np.mean(data, axis=0)  # [4, 5, 6]
mean_axis1 = np.mean(data, axis=1)  # [2, 5, 8]

# Min, max, argmin, argmax
minimum = np.min(data)      # 1
maximum = np.max(data)      # 9
min_index = np.argmin(data) # 0
max_index = np.argmax(data) # 8
```

### Practical Example: Linear Algebra

```python
# Solving linear system: Ax = b
A = np.array([[3, 1], [1, 2]])
b = np.array([9, 8])

# Solution: x = A^(-1) * b
x = np.linalg.solve(A, b)
print(f"Solution: x = {x}")  # [2, 3]

# Verify solution
print(f"Verification: Ax = {A @ x}")  # [9, 8]
```

### Working with Random Numbers

```python
# Set seed for reproducibility
np.random.seed(42)

# Generate random data
random_data = np.random.normal(0, 1, 1000)  # Normal distribution
uniform_data = np.random.uniform(0, 1, 100)  # Uniform distribution

# Random sampling
indices = np.random.choice(len(random_data), size=10, replace=False)
sample = random_data[indices]
```