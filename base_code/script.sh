for level in 1 2 3 
do
  for s in max avg 
  do
    for budget in 10 100 1000 2000
    do  
      for exp in 1 2 3 
      do  
        echo ./pacman $level ai $s $budget >> acc_output.txt 
        ./pacman $level ai $s $budget 
        cat ./output.txt >> acc_output.txt
      done
    done
  done
done
