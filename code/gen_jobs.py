import itertools
J=[]
small={'warehouse_small':[50,100,150,200,250,300,350,400],'kiva':[50,100,150,200,250,300,350,400],'sortation_small':[50,100,200,300,400,500,600],'random-32-32-20':[50,100,150,200,250,300]}
planners={'PIBT':'--planner pibt','H-PIBT':'--planner hpibt','WPP':'--planner wpp','CA-PIBT':'--planner capibt',
          'CA-PIBT-noRecover':'--planner capibt --recover 0','CA-PIBT-noHighway':'--planner capibt --cahw 0','H-PIBT+Recover':'--planner capibt --alpha 0 --gamma 0'}
for m,sizes in small.items():
    for n in sizes:
        for lab,arg in planners.items():
            for seed in range(1,6):
                J.append(f"./sim --map ../maps/{m}.map {arg} --label {lab} --agents {n} --steps 1000 --seed {seed} --out ../results/main.csv >/dev/null")
# delay robustness
for m in ['warehouse_small','kiva']:
    for n in [200,300]:
        for lab in ['PIBT','H-PIBT','WPP','CA-PIBT']:
            for dl in [0.05,0.1,0.2]:
                for seed in range(1,6):
                    J.append(f"./sim --map ../maps/{m}.map {planners[lab]} --label {lab} --agents {n} --steps 1000 --seed {seed} --delay {dl} --out ../results/delay.csv >/dev/null")
# large scale (slow ones last)
for n in [1000,2000,3000,4000]:
    for lab in ['PIBT','H-PIBT','CA-PIBT']:
        for seed in range(1,4):
            J.append(f"./sim --map ../maps/warehouse_large.map {planners[lab]} --label {lab} --agents {n} --steps 1000 --seed {seed} --out ../results/large.csv >/dev/null")
open('jobs_all.txt','w').write('\n'.join(J)+'\n'); print(len(J))
