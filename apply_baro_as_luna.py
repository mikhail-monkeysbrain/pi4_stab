import shutil, sys, time
PATH = "src/optical_flow_mavlink.cpp"
src = open(PATH, encoding="utf-8").read()
E = []
E.append(("        const double lage=hl?(monoNs()-lns)*1e-6:1e9;\n",
          "        double lage=hl?(monoNs()-lns)*1e-6:1e9;\n"))
E.append(("        const double range_to_fc=(bench_height_override>0.0)?bench_height_override:lm;\n",
          "        double range_to_fc=(bench_height_override>0.0)?bench_height_override:lm;\n"))
E.append(("""        const bool use_baro_h =
          bench_height_override<=0.0 && vrf_height_baro &&
""","""        // BARO_AS_LUNA_V1: from here on every consumer of lm/hl/lage/range_to_fc
        // sees the virtual (barometric) height exactly as if TF-Luna reported it.
        // The real TF-Luna value stays available in virtual_rangefinder_shadow.csv.
        if(bench_height_override<=0.0 && vrf_height_baro &&
           std::isfinite(vrf_height_m) && vrf_height_m>=0.03){
          lm=vrf_height_m;
          strength=255;
          lns=now;
          hl=true;
          lage=0.0;
          range_to_fc=vrf_height_m;
        }

        const bool use_baro_h =
          bench_height_override<=0.0 && vrf_height_baro &&
"""))
for i,(o,n) in enumerate(E,1):
    c=src.count(o)
    if c!=1:
        print(f"ОШИБКА: фрагмент {i} найден {c} раз, нужно 1. Файл не изменён."); sys.exit(1)
for o,n in E: src=src.replace(o,n,1)
bak=PATH+".before_baro_as_luna_"+time.strftime("%Y%m%d_%H%M%S")
shutil.copy2(PATH,bak); open(PATH,"w",encoding="utf-8").write(src)
print("Готово. Бэкап:",bak,"| правок:",len(E))
