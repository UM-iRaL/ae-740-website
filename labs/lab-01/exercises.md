---
layout: default
title: "Exercises"
parent: "Lab 1"
grand_parent: Labs
nav_order: 5
has_toc: true
---

# Exercises
{: .no_toc }


## Submission

To submit your solution, you will be creating a .zip file following the exercises below and upload it on Canvas under Assignment > Lab 1: Linux, Git, Python  by **August 31 at midnight (11:59 EST)**.

{: .warning-title } 
> Late Submission
>
> Please email us if you want to submit later than the deadline. Otherwise late penalty will be applied.


## Exercises 


### Git (5 pts)


In this exercise you are required to set a git repository, for example inside your own Github namespace. You will be downloading the zip file of this repository and submit it to Canvas.


1. Create a repository for your personal submissions
   - Go to [https://github.umich.edu/YOUR_USERNAME](https://github.umich.edu/){:target="_blank"} and click on “New Project” to create a new repository (replace `YOUR_USERNAME` with your Github namespace)
   - Create a new **Private** repository and call it as your UMich username, e.g. if your Umich email is _astark@umich.edu_, call it _astark_
   - Clone the repository to `~/ae740_personal` (you will have a team submission later) running `git clone https://github.com/YOUR_USERNAME/YOUR_UNIQUENAME.git ~/ae740_personal` (replace `YOUR_UNIQUENAME` with your uniquename)
   - Create a folder called `lab1`
2. Clone [https://github.com/UM-iRaL/AE740-F25.git](https://github.com/UM-iRaL/AE740-F25.git){:target="_blank"} in a folder of your choice


### Shell (35 pts)


1. Exercise 1 - Answer to the following questions
   - Download `https://raw.githubusercontent.com/dlang/dmd/master/druntime/benchmark/extra-files/dante.txt` (try using `wget`)
   - Create a file called `exercise1.txt` in `~/ae740_personal/lab1` and answer to the following questions
     1. How many lines does it contains?
     2. How many words does it contains?
     3. How many lines are not blank?
   - Push the file to git
2. Exercise 2 - Output redirecting
  - Install `fortune-mod` using `apt`
  - After installation, type `fortune` in your terminal to see a (hopefully) interesting proverb/quote
  - Run `fortune` 5 more times and each time redirect the output to a file called `fortunes.txt` in `~/ae740_personal/lab1` (Hint: do not recreate the file 5 times - each time a new proverb should be added to the end of `fortunes.txt`)
  - Push the file to git


> **Hint**: For the first exercise you might want to use the command `wc` (Word Count).




### C++: RandomVector (40 pts)


In this exercise we will implement the class `RandomVector`.
Inside `~/vnav-personal/lab1` create a folder called `RandomVector` and copy the content from [https://github.mit.edu/VNAV2020/Labs/tree/master/lab1](https://github.mit.edu/VNAV2020/labs/tree/master/lab1){:target="_blank"}.


The class `RandomVector` defined in the header file `random_vector.h` abstract a vector of doubles.
You are required to implement the following methods:


- `RandomVector(int size, double max_val = 1)` (constructor): initialize a vector of doubles of size `size` with random values between 0 and `max_val` (default value 1)
- `double mean()` returns the mean of the values in random vector
- `double max()` returns the max of the values in random vector
- `double min()` returns the min of the values in random vector
- `void print()` prints all the values in the random vector
- `void printHistogram(int bins)` computes the histogram of the values using `bins` number of bins between `min()` and `max()` and print the histogram itself (see the example below).


To to so complete all the `TODO`s in the file `random_vector.cpp`. When you are done compile the application by running


```bash
g++ -std=c++11 -Wall -pedantic -o random_vector main.cpp random_vector.cpp
```


**Note:** we expect you to not use the function from the `<algorithm>` header.


If you complete correctly the exercise you should see something like


```bash
$ ./random_vector
0.458724 0.779985 0.212415 0.0667949 0.622538 0.999018 0.489585 0.460587 0.0795612 0.185496 0.629162 0.328032 0.242169 0.139671 0.453804 0.083038 0.619352 0.454482 0.477426 0.0904966
Mean: 0.393617
Min: 0.0667949
Max: 0.999018
Histogram:
***     ***
***     ***
***     ***
***     ***
***     ***
***     ***
***     *** ***
*** *** *** *** ***
```


> **Optional (10 pts)**: Try to implement the methods **with and without** the functions available in the header `<algorithm>`.




---


### Python: RandomVector (40 pts)

In this exercise, you will implement a Python class called `RandomVector`.
Inside `~/ae740_personal/lab1`, create a folder called `RandomVector` and create a new file called `random_vector.py`.

The class `RandomVector` will handle a list of random floating-point numbers.
You are required to implement the following methods:

- `__init__(self, size, max_val=1.0)` (constructor): Initialize a list of size `size` with random values between 0 and `max_val` (default value 1.0)
- `mean(self)`: Returns the mean of the values in the random vector
- `max(self)`: Returns the maximum value in the random vector
- `min(self)`: Returns the minimum value in the random vector
- `print(self)`: Prints all the values in the random vector
- `print_histogram(self, bins)`: Computes the histogram of the values using `bins` number of bins between `min()` and `max()` and prints the histogram (see the example below)

Create a file called `random_vector.py` with your implementation. When you're done, create another file called `main.py` to test your implementation.

**Note:** We expect you to not use built-in functions like `min()`, `max()`, `sum()` or libraries like NumPy or Pandas for your implementation. Write the algorithms yourself.

Here's a template for your `random_vector.py` file:

```python
import random

class RandomVector:
    def __init__(self, size, max_val=1.0):
        # TODO: Initialize a list of random values between 0 and max_val
        pass
    
    def mean(self):
        # TODO: Calculate and return the mean
        pass
    
    def max(self):
        # TODO: Find and return the maximum value
        pass
    
    def min(self):
        # TODO: Find and return the minimum value
        pass
    
    def print(self):
        # TODO: Print all values in the vector
        pass
    
    def print_histogram(self, bins):
        # TODO: Create and print a text histogram with the specified number of bins
        pass
```

And here's a template for `main.py`:

```python
from random_vector import RandomVector

def main():
    # Create a RandomVector of size 20
    rv = RandomVector(20)
    
    # Print all values
    rv.print()
    
    # Print statistics
    print(f"Mean: {rv.mean():.6f}")
    print(f"Min: {rv.min():.6f}")
    print(f"Max: {rv.max():.6f}")
    
    # Print histogram with 8 bins
    print("Histogram:")
    rv.print_histogram(8)

if __name__ == "__main__":
    main()
```

Run your program with:
```bash
python main.py
```

If you complete the exercise correctly, you should see output similar to:

```bash
$ python main.py
0.458724 0.779985 0.212415 0.066795 0.622538 0.999018 0.489585 0.460587 0.079561 0.185496 0.629162 0.328032 0.242169 0.139671 0.453804 0.083038 0.619352 0.454482 0.477426 0.090497
Mean: 0.393617
Min: 0.066795
Max: 0.999018
Histogram:
***     ***
***     ***
***     ***
***     ***
***     ***
***     ***
***     *** ***
*** *** *** *** ***
```

> **Optional (10 pts)**: After completing your manual implementation, create a second version called `random_vector_builtin.py` that uses Python's built-in functions and/or libraries like NumPy. Compare the code complexity and performance between your implementations.



---

## Transfer ownership of Git repository


If you created the repository in your personal account instead of _VNAV2020-submissions_ you might want to transfer the ownership in order to complete your submission.


1. On GitHub, navigate to the main page of the repository.
2. Under your repository name, click **Settings**.  
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep1.png' | relative_url}})
3. Scroll down until your reach the **Danger Zone**, then click **Transfer**.  
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep2.png' | relative_url}})
4. Type the name of your repository in the first row and _VNAV2020-submissions_ in the second, then click **I understand, transfer this repository**.  
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep3.png' | relative_url}})
5. Done!
