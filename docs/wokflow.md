# Working with git and using github

For this project we will be using github and github actions to ensure that the every ones contributions follows the same standards
and that the code is tested.

## [Issues](https://github.com/ivario123/E7012E_software/issues)

While issues sounds like something has to be wrong it is merely a mechanism for tracking discussions or tasks.
A task should be self contained, i.e. it can be worked on without needing to complete parts of any other tasks.
The tasks can, however depend on other tasks thus forcing us to complete them in order and giving us a clear roadmap.
If a task depends on another task this should be clearly marked in the issue thereby allowing us to pick the
blocking tasks first. Everyone can and should open issues when they have a feature they want to include or if there
is anything they want to discuss.

### Using issues as discussion forums

The issues can also be used as discussion forums, say that we want to decide on a language to use for the computer vision tasks,
while this can be discussed in another forum it might be good to be able to see our reasoning for this choice in the future if we
ever change our mind.

## [github projects](https://github.com/users/ivario123/projects/4)

This is a simple overview of the tasks and what groups they fit in to, this allows us to plan out issues and distribute them in an efficient manner.
It also allows us to sort the tasks in to so called stories, while this might not be needed it is a nice to have.
A story is a self contained feature request that describes a usage example, say we want to be able to plan a path using the cameras alone,
this might contain multiple tasks, say line extraction from a video feed, distance estimation from a video feed, model predictive control and so on.

## [Working with git](https://github.com/git-guides/install-git)

To keep track of changes we will be using a tool called git. It is a "version control" system that keeps track of the differences in between to different versions of a file.
Allowing you to either reconstruct a file tree from a commit or merge two different version of the same file. Say that we have a single file

- `file.txt`

```txt
Here is some text.
```

And that person A wants to add a paragraph about birds,

```txt
Here is some text.

Here is some other text about birds.
```

This is well and all. If person A now want to send this to the main repository it simply writes

```bash
# Include the changes to file.txt
git add file.txt # One can also write git add . to include all files in the folder

# Add a nice message about what you modified
git commit -m "added a short text about birds"

# Send the changes off to the repository
git push
```

And this works fine, but lets say that person B now has the first version of the file, i.e.

```txt
Here is some text.
```

And wants to add some text about dogs.

```txt
Here is some text.

Here is some other text about dogs.
```

Then tries to send this change to the server

```bash
# Include the changes to file.txt
git add file.txt # One can also write git add . to include all files in the folder

# Add a nice message about what you modified
git commit -m "added a short text about dogs"

# Send the changes off to the repository
git push
```

But this time they will get an error since the file history on the server and on person Bs computer now do not match.
The solution to this problem is using `branches`, a branch is a diverging history that contains some changes that are not in the main history.
These branches have names, and they should be used to convey meaning about what changes you are making,
so if person a was to create a branch `birds` before committing their changes

```bash
# Create a new branch
git checkout -b "birds"

# Include the changes to file.txt
git add file.txt # One can also write git add . to include all files in the folder

# Add a nice message about what you modified
git commit -m "added a short text about birds"

# Send the changes off to the repository and create
# the same branch on the server
git push origin birds
```

Person B can then do a similar thing

```bash
# Create a new branch
git checkout -b "dogs"

# Include the changes to file.txt
git add file.txt # One can also write git add . to include all files in the folder

# Add a nice message about what you modified
git commit -m "added a short text about dogs"

# Send the changes off to the repository and create
# the same branch on the server
git push origin dogs
```

We can then [`pull`](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) person A's changes in to the main history and then include these changes in person B's history by doing

```bash
# Bring in the latest changes from the server
git fetch

# Change the starting point of dogs to be the latest
# version of the servers main history
git rebase origin/main
```

in person B's file history. This will require you to resolve some merge conflicts, these are simply which parts of person A's history do you want to keep and what parts of person B's history we keep. After this [`rebase`](https://www.atlassian.com/git/tutorials/rewriting-history/git-rebase) we can create another [`pull request`](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)
to merge person B's changes in to the main history.
Now both changes to the file will be included.

### [Continuous testing](https://docs.github.com/en/actions/using-workflows)

To ensure that all changes follow the same patterns and produce the expected results we use continuous testing, this means that as soon as a pull request is
opened ( in to main ) the server runs a set of tests that ensure that we follow the same patters everywhere in the code and that any and all tests in the code
base pass with flying colors. This allows the reviewers to be more sure that the code is correct before merging it in to the main history, and hopefully
it means that the code in the main branch always works and looks good.

## Pull-requests

To include any changes in the main branch you need to open a pull request that describes what you want to do, if you are not certain on how to write one of these
please look at the [pr_template](./pr_template.md) which provides a simple example of what you should include.
Moreover, the PR needs to pass all tests and be reviewed by at least one other contributor before merging in to the main branch, thus forcing more people
to look over every bit of code acting like a safeguard both of system understanding and code quality.

## Issues

For a short reference on what to include in new issues please have a look at the [issue template](./issue_template.md).
It gives a brief introduction in to what we should include in the issues.

## Tooling

### Embedded

For the embedded side we will be using [rust](https://docs.rust-embedded.org/book/)/[rtic](https://rtic.rs/2/book/en/) on the [nrf52840](https://github.com/nrf-rs/nrf-hal) development kit. The reason for using the nrf52840 instead of the Arduino Due is the nice rust support and the groups prior experience with the chip in question.

An embedded rust program can be split in to a few abstraction levels

```zsh
| hardware | pac | hal | rtic | user code
```

The hardware is the nrf52840 chip. Instead of accessing the chip via pointers and raw memory addresses like we would using c/c++ we use something called a [PAC](https://docs.rust-embedded.org/book/start/registers.html) which is short for peripheral access crate, this is an auto generated file that provides us with nice names
and simple abstractions over the hardware register ensuring that bit accesses are performed in a correct manner. On top of these register accesses that the PAC provides people write so called [`hal`](https://dev.to/apollolabsbin/what-the-hal-the-quest-for-finding-a-suitable-embedded-rust-hal-2i02)'s which is short for hardware abstraction layer, these allow us to use the peripherals a compact and pre defined manner, removing a lot of error cases and allowing the programmer to spend
more time solving the problem at hand rather than reading the processors documentation. Then we have the [`rtic`](https://rtic.rs/2/book/en/) abstraction,
this abstraction allows us to write simple interrupt handlers that execute a piece of code when ever a specific event is triggered, and treat these as individual "threads"/"tasks" ( in rtic there are no threads ) which have a specific priority and a set of either shared or local data that should persist between context switching.
Finally we have the code that we write, this code defines what the processor should do and when.

#### Logging

The logging on embedded (realtime) systems is tricky thing, we want to be able to trace the execution of a program while not interfering with the timings/execution of the system.
A common way of writing data from the [mcu](https://en.wikipedia.org/wiki/Microcontroller) to the host system is using [rtt](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) to send full strings of text from the micro controller to the host system. A more modern approach to this
is the [defmt](https://defmt.ferrous-systems.com/) library which sends the data to be formatted to the host machine and formats it there rather than formatting the data
on the embedded system, thus reducing the time spent on writing debug data to over the debugger.

### Hosted

The "hosted" (code that runs with an operating system) is t.b.d.

### Shared

The main benefit of running rust on the host side and the embedded side is that we can share encodings for different types of data
on the host machine and the embedded system. This allows us to write a library (shared) that contains all functionality that the both systems have.

## Testing

In rust it is really easy to write tests, they are simply modules that are not build unless we run

```bash
cargo test
```

A test is written as follows

```rust
// cfg test says that we only build the following code if
// we build with the test flag i.e. cargo test
#[cfg(test)]
// module named test, can be what ever, I just like the name test
mod test {

    // Defines all tests for this file
    #[test]
    fn test_add(){
        let a = 1+2;
        assert!(a == 1+2);
    }

    #[test]
    fn test_sub(){
        let b = 1-2;
        assert!(b == 1-2);
        assert!(b != 1+2)
    }
}
```

All functionality that can be ran in isolation should be written in such a way that it is testable, this means that the code to extract lines
from an image should not be written as

```rust
pub fn extract_lines() -> Lines {
    let image = GetImage();
    for ... in ... {
        for ... in  ... {
            ...
        }
    }
    return lines
}
```

As this function is not testable without replacing the GetImage function, we should instead write it like

```rust
pub fn extract_lines(image:Image) -> Lines {
    for ... in ... {
        for ... in  ... {
            ...
        }
    }
    return lines
}
```

Which can then easily be tested like

```rust
#[cfg(test)]
mod test{
    use super::extract_lines;

    /// Note that we probably should have more tests than one here
    /// as there are a lot of different lines that we can find in an image.
    fn test_extract_lines(){
        let image = SomeTestImage;
        let result = extract_lines(image.clone());

        let expected = Lines {....};

        assert!(result == expected);
    }
}
```

This ensures that no future merges breaks the extract_lines function as it will be tested before any future merges.
