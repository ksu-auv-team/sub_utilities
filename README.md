# sub-utilities
This repo serves as the top level repo for KSU's AUV autonomy. It utilizes something called [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). These submodules are a way to organize and modularize our code to be easily editable and deployable to multiple systems. Basically, it's another repo inside a repo.

Because of this submodule system, I would recomend cloning with the following:
```
git clone --recurse-submodules https://github.com/ksu-auv-team/sub-utilities.git
```
This will clone the repo and initialize the submodules automatically. 

If you have not cloned this way, you may also:
```
git submodule init
git submodule update
```

I can't stress enough how much a brief look through [this submodules tutorial](https://git-scm.com/book/en/v2/Git-Tools-Submodules) will help you if you are confued.

### Scripts
This repo also has nice startup scrips to aid in the useability of the submarine all located in the `scripts` directory. 

* `pict.py` will publish camera footage on the `imgs` topic from /dev/video0
* `vis.py` allows the user to listen on the `imgs` topic and show realtime video feed.
* `runsub.py` will run the startup process for all of AUV's different nodes including: roscore, movement_package, execute_witState, etc. This is the script you want to run if you are trying to run the full machine. 

### Submodules
Inside the `submodules` directory is where all the submodules of this repo live. Most of these are also required for making the sub work properly and you should checkout their README's to get a better understanding of what they all do.   

If you push a commit to a submodule, it will not ***Automatically*** show up here. You need to go into the `submodules/<your-updated-submodule>` directory, do a `git fetch`, then checkout the commit that you want. When you then return to the base `sub-utilities` directory, and do a `git status`, you should see it showing that you updated a submodule. You can then `git add` and `git commit` that like normal and it will update the `sub-utilities` repo.  
