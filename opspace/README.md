Operational-Space Task Library for Human-Centered Robotics, UT Austin
=====================================================================

This is a subtree of the [Whole-Body Control for Human-Centered
Robotics][utaustin-wbc] project. Please check that out, and read [this
tutorial][tut] to understand what `git-subtree` can do for us, and how
to use it for development.

[utaustin-wbc]: https://github.com/poftwaresatent/utaustin-wbc
[tut]: http://psionides.jogger.pl/2010/02/04/sharing-code-between-projects-with-git-subtree/

Quick'n'Dirty git-subtree instructions
--------------------------------------

1. Install [git-subtree](https://github.com/apenwarr/git-subtree) and
   read its man page.

2. To pull this into an umbrella project, do something like this
   (adjust the URL depending on whether you have write access or not,
   replacing it with your own fork, or suchlike):

        git remote add opspace git@github.com:poftwaresatent/utaustin_wbc_opspace.git
        git fetch opspace
        git subtree add -P opspace opspace/master

3. If you later want to pull opspace changes into your umbrella project:

        git fetch opspace
        git subtree merge -P opspace opspace/master

4. After you've made some changes to opspace while working on the
   umbrella project, use a separate umbrella branch to push them back
   to the opspace master. E.g. with a disposable branch called `foo`:

        git subtree split -P opspace -b foo
        git push opspace foo:master
        git branch -D foo

5. Feel free to improvise and update this file... using `git-subtree`
   instead of `git-submodule` is an experiment motivated by some
   frustration with the latter.
