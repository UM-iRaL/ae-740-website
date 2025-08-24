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

To submit your solution, you will be creating a .zip file following the exercises below and upload it on Canvas under Assignment > Lab 1: Linux, Git, Python  by **September 1 at midnight (11:59 EST)**.

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
<!-- 2. Clone [https://github.com/UM-iRaL/AE740-F25.git](https://github.com/UM-iRaL/AE740-F25.git){:target="_blank"} in a folder of your choice -->


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

---


### Python: RandomVector (40 pts)

In this exercise, you will implement a Python class called `RandomVector`. In the `ae_740_personal` directory, create a Python virtual environment `ae740_venv`, that we will use throughout the course. Inside `~/ae740_personal/lab1`, create a folder called `RandomVector` and clone the contents from [https://github.com/UM-iRaL/ae740_labs/tree/main/lab1](https://github.com/UM-iRaL/ae740_labs/tree/main/lab1). This contains two files -- 
- `random_vector.py`: Class definition.
- `main.py`: Main Python file, that imports the `RandomVector` class.

The class `RandomVector` will handle a list of random floating-point numbers.
You are required to implement the following methods inside `random_vector.py`:

- `__init__(self, size, max_val=1.0)` (constructor): Initialize a list of size `size` with random values between 0 and `max_val` (default value 1.0)
- `mean(self)`: Returns the mean of the values in the random vector
- `max(self)`: Returns the maximum value in the random vector
- `min(self)`: Returns the minimum value in the random vector
- `print(self)`: Prints all the values in the random vector
- `print_histogram(self, bins)`: Computes the histogram of the values using `bins` number of bins between `min()` and `max()` and prints the histogram (see the example below)

When you're done, create another file called `main.py` to test your implementation.

**Note:** We expect you to not use built-in functions like `min()`, `max()`, `sum()` or libraries like NumPy or Pandas for your implementation. Write the algorithms yourself.

If you complete the exercise correctly, when you run `python main.py` you should see output similar to:

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