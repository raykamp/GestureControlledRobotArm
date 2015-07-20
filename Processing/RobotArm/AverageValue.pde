public class AverageValue {
    private int numberOfEntries;
    private int value;
    public AverageValue() {
        numberOfEntries = 0;
        value = 0;
    }
    public void addValue(int entry){
      value += entry;
      numberOfEntries++;
    }
    public int getAverageAndReset(){ // 
      int temp = value/numberOfEntries;
      value = 0;
      numberOfEntries = 0;
      return temp;
    }
}
