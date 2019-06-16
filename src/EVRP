
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

import nju.lzx.Constraint.*;
import nju.lzx.Data.*;
import nju.lzx.Algorithm.*;
import nju.lzx.Interface.*;
import nju.lzx.LocalSearchOperator.*;
import nju.lzx.InitialSolutionOperator.*;
import nju.lzx.Utility.*;
import nju.lzx.Route.*;
import nju.lzx.LabelSetting.*;


// TODO: Auto-generated Javadoc
/**
 * Electric Vehicle Routing Problem (EVRP)求解示例。 (目标是最小化总成本。).
 */
public class EVRP {

	
	/**
	 * EVRP算例。
	 */
	public static class InstanceElectric extends Instance{

		/** 首个充电站编号。 */
		public int u;
		
		/** 顾客货物的重量。 */
		public double[] w;
		
		/** 顾客货物的体积。 */
		public double[] v;

		/** 车型种类。 */
		public int o; 
		
		/** 车辆重量容量。 */
		public double[] W;
		
		/** 车辆体积容量。 */
		public double[] V;
		
		/** 车辆满电的最大行驶里程。 */
		public double[] R;
		
		/** 车辆每公里行驶成本。 */
		public double[] c;
		
		/** 车辆的固定成本。 */
		public double[] f;
		
		/** 两个顾客点之间可以经过的充电站集合。 */
		public ArrayList<ArrayList<ArrayList<Integer>>> evs;
	}
	
	/**
	 * 主函数。.
	 *
	 * @param args 参数。
	 * @throws IOException IO异常。
	 */
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		double t1 = System.nanoTime();
		//InstanceElectric inst = (InstanceElectric) load_instance("../data/small/input_node.txt", "../data/small/input_vehicle_type.txt", "../data/small/input_distance-time.txt");
		InstanceElectric inst = (InstanceElectric) load_instance("../data/input_node.txt", "../data/input_vehicle_type.txt", "../data/input_distance-time.txt");
		double t2 = System.nanoTime();
		double load_instance_time = (t2 - t1) / 1e9;
		////设置模式参数
		inst.parameter.Mode.multi_trip_enable = true;
		inst.parameter.Mode.multi_thread_enable = false;
		System.setProperty("java.util.concurrent.ForkJoinPool.common.parallelism", "8");
		//设置初始解参数
		inst.parameter.InitialSolution.log_print = false;
		//设置禁忌搜索参数
		inst.parameter.TabuSearch.maximum_iteration = 1000000;
		inst.parameter.TabuSearch.maximum_tabu_tenure = 500;
		inst.parameter.TabuSearch.tenure_decay_rate = 0.99;
		inst.parameter.TabuSearch.mininum_tabu_tenure = 10;
		inst.parameter.TabuSearch.mininum_shake_tenure = 300;
		inst.parameter.TabuSearch.minimum_shake_iteration = 100;
		inst.parameter.TabuSearch.label_setting_tenure = 500;
		inst.parameter.TabuSearch.log_print = true ;
		inst.parameter.TabuSearch.log_detail = false;
		//设置算子参数
		inst.parameter.Operator.insertion_prune_threshhold = 1e6;
		inst.parameter.Operator.exchange_prune_threshhold = 1e6;
		inst.parameter.Operator.cross_prune_threshhold = 1e6;
		inst.parameter.Operator.remove_prune_threshhold = -1e6;
		inst.parameter.Operator.route_cross_threshhold = 100;
		
		
		//构造约束条件
		Constraint[] cnts = new Constraint[7];
		MinimizeDistance.ConstraintData[] dist_dats = new MinimizeDistance.ConstraintData[inst.o];
		for(int i = 0; i < inst.o; i++)
			dist_dats[i] = new MinimizeDistance.ConstraintData(inst.d, inst.c[i]);
		cnts[0] = new MinimizeDistance(dist_dats, 0, 1);
		
		MinimizeChargeCost.ConstraintData ch_dat = new MinimizeChargeCost.ConstraintData(inst.u, 50);
		cnts[1] = new MinimizeChargeCost(ch_dat, 1);
		
		MinimizeFixedCost.ConstraintData[] fixed_dats = new MinimizeFixedCost.ConstraintData[inst.o];
		for(int i = 0; i < inst.o; i++)
			fixed_dats[i] = new MinimizeFixedCost.ConstraintData(inst.f[i]);
		cnts[2] = new MinimizeFixedCost(fixed_dats, 0, 1);
		
		CapacityConstraint.ConstraintData[] capw_dats = new CapacityConstraint.ConstraintData[inst.o];
		for(int i = 0; i < inst.o; i++)
			capw_dats[i] = new CapacityConstraint.ConstraintData(inst.w, inst.W[i]);
		cnts[3] = new CapacityConstraint(capw_dats, 0, true, 1000);
		
		CapacityConstraint.ConstraintData[] capv_dats = new CapacityConstraint.ConstraintData[inst.o];
		for(int i = 0; i < inst.o; i++)
			capv_dats[i] = new CapacityConstraint.ConstraintData(inst.v, inst.V[i]);
		cnts[4] = new CapacityConstraint(capv_dats, 0, true, 500);
		
		ElectricCapacityConstraint.ConstraintData[] ecap_dats = new ElectricCapacityConstraint.ConstraintData[inst.o];
		for(int i = 0; i < inst.o; i++)
			ecap_dats[i] = new ElectricCapacityConstraint.ConstraintData(inst.u, inst.R[i], inst.d);
		cnts[5] = new ElectricCapacityConstraint(ecap_dats, 0, true, 50);
		
		TimeWindowConstraint.ConstraintData tw_dat = new TimeWindowConstraint.ConstraintData(inst.e, inst.l, inst.t, inst.s);
		cnts[6] = new TimeWindowConstraint(tw_dat, true, 50);
		
		//构造需要访问的节点集合
		ArrayList<Atr> atrs = new ArrayList<Atr>();
		for(int i = 1; i < inst.u; i++){
			atrs.add(new Atr(i));
		}
		boolean[] exc = new boolean[inst.n];
		for(int i = 1; i < inst.u; i++)
			exc[i] = true;
		
		//构造算法算子
		Operator[] operators = new Operator[3];
		double[] coefs = new double[3];
		operators[0] = new RelocateBase(inst, exc);
		coefs[0] = 1;
		operators[1] = new ExchangeBaseDeep(inst, exc);
		coefs[1] = 1;
		operators[2] = new CrossBase(inst, exc);
		coefs[2] = 1;
		
		//构造初始解
		
		int[] tp = new int[inst.o];
		int[] mt = new int[inst.o];
		for(int i = 0; i < inst.o; i++){
			tp[i] = i;
			mt[i] = 1000;
		}
		GreedyGeneral greedy = new GreedyGeneral(inst, new GreedyGeneral.AlgorithmData(0, inst.o, tp, mt), 0, 0, new InsertElectric(inst, new InsertElectric.OperatorData(inst.u, inst.evs), cnts));
		
		ArrayList<Route> s = greedy.generate(atrs);
		s = toDeep(inst, s);
		
		//ArrayList<Route> s = load_solution(inst, cnts, "result/result.txt");
		System.out.println(greedy.toString(s));
		greedy.check(s, true, exc);
		System.out.println("feasibility of the initial solution>>>" + greedy.is_feasible(s) + "\t" + s.size() + "\t" + greedy.get_total_cost(s));
		double t3 = System.nanoTime();
		double initial_construct_time = (t3 - t2) / 1e9;
		System.out.println("load instance time>>>" + load_instance_time + "\t initial construct time>>>" + initial_construct_time);
		//System.exit(0);
		 
		
		//优化解的成本
		EVSOpt.AlgorithmData algo_data = new EVSOpt.AlgorithmData(inst.R, inst.d, inst.u, inst.evs);
		TabuSearch tabu = new TabuSearch(inst, operators, coefs, new EVSOpt(inst, algo_data));
		ArrayList<Route> bs = tabu.solve(s);

		tabu.check(bs, true, exc);
		//System.out.println(tabu.toString(bs));
		double t4 = System.nanoTime();
		System.out.println("computation time>>>" + (t4 - t1) / 1e9);
		inst.statistics.print();
		write_solution("result/c1_10_8_1.txt", bs);
	}
	
	/**
	 * S 2 t.
	 *
	 * @param t the t
	 * @return the double
	 */
	public static double s2t(String t){
		String[] sub = t.split(":");
		int hour = Integer.parseInt(sub[0]);
		int min = Integer.parseInt(sub[1]);
		if(hour == 0 && min == 0)
			hour = 24;
		return hour * 60 + min;
	}

	
	/**
	 * 加载算例。.
	 *
	 * @param node_file 节点信息文件路径。
	 * @param vehicle_file 车辆信息文件路径。
	 * @param dist_file 距离信息文件路径。
	 * @return 返回算例。
	 * @throws FileNotFoundException IO异常。
	 */
	public static Instance load_instance(String node_file, String vehicle_file, String dist_file) throws FileNotFoundException{
		//node information
		InstanceElectric inst = new InstanceElectric();
		Scanner cin = new Scanner(new BufferedReader(new FileReader(node_file)));
		cin.nextLine();
		ArrayList<Double> lng_set = new ArrayList<Double>();
		ArrayList<Double> lat_set = new ArrayList<Double>();
		ArrayList<Double> w_set = new ArrayList<Double>();
		ArrayList<Double> v_set = new ArrayList<Double>();
		ArrayList<Double> e_set = new ArrayList<Double>();
		ArrayList<Double> l_set = new ArrayList<Double>();
		inst.u = 0;
		while(cin.hasNextLine()){
			String line = cin.nextLine();
			String[] sub = line.split("\t");
			int type = Integer.parseInt(sub[1]);
			double lg = Double.parseDouble(sub[2]);
			double lt = Double.parseDouble(sub[3]);
			lng_set.add(lg);
			lat_set.add(lt);
			if(type == 1){
				inst.u++;
				w_set.add(0.0);
				v_set.add(0.0);
				e_set.add(s2t(sub[6]));
				l_set.add(s2t(sub[7]));
			}
			else if(type == 2){
				inst.u++;
				w_set.add(Double.parseDouble(sub[4]));
				v_set.add(Double.parseDouble(sub[5]));
				e_set.add(s2t(sub[6]));
				l_set.add(s2t(sub[7]));
			}
			else{
				w_set.add(0.0);
				v_set.add(0.0);
				e_set.add(e_set.get(0));
				l_set.add(l_set.get(0));
			}
		}
		inst.n = lng_set.size();
		inst.lng = new double[inst.n];
		inst.lat = new double[inst.n];
		inst.w = new double[inst.n];
		inst.v = new double[inst.n];
		inst.e = new double[inst.n];
		inst.l = new double[inst.n];
		inst.s = new double[inst.n];
		for(int i = 0; i < inst.n; i++){
			inst.lng[i] = lng_set.get(i);
			inst.lat[i] = lat_set.get(i);
			inst.w[i] = w_set.get(i);
			inst.v[i] = v_set.get(i);
			inst.e[i] = e_set.get(i);
			inst.l[i] = l_set.get(i);
			if(i < 1){
				inst.s[i] = 60;
			}
			else if(i < inst.u){
				inst.s[i] = 30;
			}
			else{
				inst.s[i] = 30;
			}
		}
		cin.close();
		//distance information
		inst.d = new double[inst.n][inst.n];
		inst.t = new double[inst.n][inst.n];
		cin = new Scanner(new BufferedReader(new FileReader(dist_file)));
		cin.nextLine();
		while(cin.hasNextLine()){
			String line = cin.nextLine();
			String[] sub = line.split(",");
			int from = Integer.parseInt(sub[1]);
			int to = Integer.parseInt(sub[2]);
			int dist = Integer.parseInt(sub[3]);
			int time = Integer.parseInt(sub[4]);
			inst.d[from][to] = inst.d[to][from] = dist;
			inst.t[from][to] = inst.t[to][from] = time;
		}
		cin.close();
		//vehicle information
		cin = new Scanner(new BufferedReader(new FileReader(vehicle_file)));
		ArrayList<String> lines = new ArrayList<String>();
		cin.nextLine();
		while(cin.hasNextLine()){
			lines.add(cin.nextLine());
		}
		cin.close();
		inst.o = lines.size();
		inst.W = new double[inst.o];
		inst.V = new double[inst.o];
		inst.R = new double[inst.o];
		inst.c = new double[inst.o];
		inst.f = new double[inst.o];
		for(int i = 0; i < inst.o; i++){
			String line = lines.get(i);
			String[] sub = line.split("\t");
			inst.V[i] = Double.parseDouble(sub[2]);
			inst.W[i] = Double.parseDouble(sub[3]);
			inst.R[i] = Integer.parseInt(sub[5]);
			inst.c[i] = Double.parseDouble(sub[7]) / 1000;
			inst.f[i] = Double.parseDouble(sub[8]);
		}

		inst.evs = new ArrayList<ArrayList<ArrayList<Integer>>>();
		
		for(int i = 0; i < inst.u; i++){
			inst.evs.add(new ArrayList<ArrayList<Integer>>());
			for(int j = 0; j < inst.u; j++){
				ArrayList<Integer> set = new ArrayList<Integer>();
				if(i != j){
					for(int k = inst.u; k < inst.n; k++){
						double arrive = 0;
						if(i == 0)
							arrive = inst.e[i] + inst.t[i][k] + inst.s[k] + inst.t[k][j];
						else
							arrive = inst.e[i] + inst.s[i] + inst.t[i][k] + inst.s[k] + inst.t[k][j];
						if(arrive > inst.l[j])
							continue;
						boolean dominated = false;
						for(int h = 0; h < set.size(); h++){
							int id = (int) set.get(h);
							if(inst.d[i][id] <= inst.d[i][k] && inst.d[id][j] <= inst.d[k][j] && inst.t[i][id] + inst.s[id] + inst.t[id][j] <= inst.t[i][k] + inst.s[k] + inst.t[k][j]){
								dominated = true;
								break;
							}
						}
						if(dominated)
							continue;
						for(int h = 0; h < set.size(); h++){
							int id = (int) set.get(h);
							if(inst.d[i][id] >= inst.d[i][k] && inst.d[id][j] >= inst.d[k][j] && inst.t[i][id] + inst.t[id][j] >= inst.t[i][k] + inst.t[k][j]){
								set.remove(h);
								h--;
							}
						}
						set.add(k);
					}
				}
				inst.evs.get(i).add(set);
			}
		}
		return inst;
	}
	
	/**
	 * 将一个普通的解转换成一个可以被ExchangeBaseDeep和RelocateBseIntra操作的解，即路径中包含子路径。.
	 *
	 * @param inst 算例信息。
	 * @param s 当前解。
	 * @return 返回新的解。
	 */
	public static ArrayList<Route> toDeep(Instance inst, ArrayList<Route> s){
		ArrayList<Route> ns = new ArrayList<Route>();
		for(int i = 0; i < s.size(); i++){
			Route r = s.get(i);
			Reference ref = r.get_reference();
			Route nr = new RouteBase(inst, ref.len, ref.seq, false, true);
			Constraint[] _cnts = r.get_constraints();
			Constraint[] _cnts2 = new Constraint[_cnts.length];
			for(int j = 0; j < _cnts.length; j++){
				_cnts2[j] = _cnts[j].copy(nr.get_reference());
			}
			nr.add_constraints(_cnts2);
			nr.set_type(r.get_type());
			nr.check();
			ns.add(nr);
		}
		return ns;
	}
	
	
	
	/**
	 * 将解保存到文件。.
	 *
	 * @param file 文件名称。
	 * @param s 当前解。
	 * @throws IOException I/O异常。
	 */
	public static void write_solution(String file, ArrayList<Route> s) throws IOException{
		BufferedWriter out = new BufferedWriter(new FileWriter(file));
		double total_cost = 0;
		for(int i = 0; i < s.size(); i++){
			total_cost += s.get(i).get_total_cost();
		}
		out.write(s.size() + " " + total_cost);
		out.newLine();
		for(int i = 0; i < s.size(); i++){
			Route r = s.get(i);
			out.write(r.get_type() + " ");
			for(int j = 0; j < r.get_size(); j++){
				int id = r.get_node(j);
				out.write(id + " ");
			}
			out.newLine();
		}
		out.close();
	}
	
	/**
	 * 从文件中加载解。.
	 *
	 * @param inst 算例信息。
	 * @param cnts 问题约束条件集合。
	 * @param path 解的文件路径。
	 * @return the array list
	 * @throws FileNotFoundException the file not found exception
	 */
	public static ArrayList<Route> load_solution(Instance inst, Constraint[] cnts, String path) throws FileNotFoundException{
		ArrayList<Route> rs = new ArrayList<Route>();
		Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));
		cin.nextLine();
		while(cin.hasNextLine()){
			String line = cin.nextLine();
			String[] subs = line.split(" ");
			int type = Integer.parseInt(subs[0]);
			int[] seq = new int[subs.length];
			for(int i = 1; i < subs.length; i++){
				seq[i] = Integer.parseInt(subs[i]);
			}
			RouteBase r = new RouteBase(inst, subs.length, seq, false, true);
			Constraint[] _cnts = new Constraint[cnts.length];
			for(int j = 0; j < cnts.length; j++){
				_cnts[j] = cnts[j].copy(r.get_reference());
			}
			r.add_constraints(_cnts);
			r.set_type(type);
			rs.add(r);
		}
		cin.close();
		return rs;
	}


}
