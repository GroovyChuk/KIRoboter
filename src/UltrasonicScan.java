import java.util.ArrayList;

public class UltrasonicScan {
	private ArrayList<Float> results;
	
	public UltrasonicScan (int stepSize) {
		this.results = new ArrayList<Float>();
	}
	
	public void addResult (float result) {
		results.add(result);
	}
	
	public ArrayList<Float> getResults () {
		return results;
	}
}
