package sim2D;

import javax.swing.JFrame;
import java.awt.Dimension;
import java.awt.Container;
import java.awt.GridLayout;

/**
 * Chris Underwood
 * undeze@gmail.com
 * 
 */
public class  simOpen  extends JFrame 
{
    public sim2 Sim2 = new sim2();
    public autoPilot AP = new autoPilot();

    public simOpen(String name){   //Constructor
        super(name);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        Dimension instrumentsSize = new Dimension(1220, 720);
        this.setSize(instrumentsSize);
    }

    public static void main(String[] args){
        simOpen instruments = new simOpen("Simulator");
        Container contentPane = instruments.getContentPane();
        contentPane.setLayout(new GridLayout(1,1));
        simGraphics1 edit = new simGraphics1(instruments.getSize());
        contentPane.add(edit);
        instruments.setVisible(true);
    }
}
