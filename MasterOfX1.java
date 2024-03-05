package master;
import robocode.*;
import robocode.util.Utils;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.util.*;


public class MasterOfX1 extends AdvancedRobot  {
	
	// maximo poder nivel 3
	final double FIREPOWER = 3;
	//tamanho do robo
	final double HALF_ROBOT_SIZE = 18;

	//escaneia os robos no mapa e 
	final Map<String, RobotData> enemyMap;

	// varredura do scanner
	double scanDir = 1;

	//ultimo robo escaneado.
	RobotData oldestScanned;

	// robos alvo, sentar o dedo
	RobotData target;

	// recebe a ultima vez que o robo mudou de direção
	long lastDirectionShift;

	// direção atual
	int direction = 1;

    // contruindo o robo, 
	public MasterOfX1() {
		// escaneia em ralação ao ultimo escaneado
		enemyMap = new LinkedHashMap<String, RobotData>(5, 2, true);
	}

	// metodo que bota o robo pra correr
	@Override
	public void run() {
		

		// inicializa o robo e todos os seus atributos
		initialize();
		setBodyColor(Color.yellow);
		setGunColor(Color.black);
		setRadarColor(Color.orange);
		setBulletColor(Color.cyan);
		setScanColor(Color.cyan);
		// Se o robô não agir, o jogo irá desabilitar
		while (true) {
			
			// escaneia os robos inimigos
			handleRadar();
			// gira a arma para o inimigo escaneado
			handleGun();
			// vira o robbo para a direção do inimigo
			moveRobot();

			// escaneia outros robos
			scan();
		}
	}

	// colisao do scaner para char outro robo
	@Override
	public void onScannedRobot(ScannedRobotEvent scannedRobotEvent) {
		// Verifique se o robô escaneado não é um robô sentinela
		if (!scannedRobotEvent.isSentryRobot()) {
			// confirma se o robo escanedado é um sentinela...

			// atualiza o mapa para identificar proxima posição do inimigo
			updateEnemyMap(scannedRobotEvent);

			// atualiza a direção do scaner
			updateScanDirection(scannedRobotEvent);

			// atualiza a posição dos inimigos
			updateEnemyTargetPositions();
		}
	}

	// informa quando outro robo morre
	@Override
	public void onRobotDeath(RobotDeathEvent robotDeathEvent) {
		// pega o nome do robo que morreu
		final String deadRobotName = robotDeathEvent.getName();

		//remove as informações do robo que morreu
		enemyMap.remove(deadRobotName);

		// remove os dados do robo mais antigo
		if (oldestScanned != null && oldestScanned.name.equals(deadRobotName)) {
			oldestScanned = null;
		}
		if (target != null && target.name.equals(deadRobotName)) {
			target = null;
		}
	}

	
	 //atualiza o robo antes de cada batalha.
	 
	private void initialize() {
		// radar e canhao independentes
		setAdjustRadarForGunTurn(true);
		setAdjustGunForRobotTurn(true);

		// adiciona cores ao robo
		setBodyColor(new Color(0x5C, 0x33, 0x17)); // Yellow
		setGunColor(new Color(0x45, 0x8B, 0x74)); // Aqua Marine
		setRadarColor(new Color(0xD2, 0x69, 0x1E)); // Orange Chocolate
		setBulletColor(new Color(0xFF, 0xD3, 0x9B)); // Burly wood
		setScanColor(new Color(0xCA, 0xFF, 0x70)); // Olive Green
	}

	/**
	 * manuzeiia o scaner para procurar o inimigo
	 */
	private void handleRadar() {
		// gira o radar infinitamente para a esquerda e para a direita
		setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
	}

	/**
	 * gira o canhao para o alvo e atira
	 */
	private void handleGun() {
		// atualiza o canhao para o alvo e atira
		updateTarget();
		// atualiza a direção do canhão
		updateGunDirection();
		// senta o dedo quando estiver pronto
		fireGunWhenReady();
	}

	/**
	 * move o robo pelo mapa
	 */
	private void moveRobot() {

		// move o robo o mais perto do inimigo
		// se nao tiver pra onde ir muda de direção
		// move de  um lado para o outro pra nao ficar parado

		int newDirection = direction;

		// chega o mais perto do robo definido como alvo
		if (target != null) {
			// calcula a distandia da borda para se manter o maximo 
			int borderRange = getSentryBorderSize() - 20;
			// se move para a horizontal e horizontal
			boolean horizontal = false;
			boolean vertical = false;

			// gera uma nova rumo
			double newHeading = getHeadingRadians();

			//verifica de esta na borda superior se sim move na horizontal
			if (getY() < borderRange || getY() > getBattleFieldHeight() - borderRange) {
				horizontal = true;
			}
			// verifica de esta nas boras laterais se sim move para a vertical
			if (getX() < borderRange || getX() > getBattleFieldWidth() - borderRange) {
				vertical = true;
			}

				//se tiver em um dos cantos nove para vertical ou horizontal.
			if (horizontal && vertical) {
		
				if (Math.abs(target.targetX - getX()) <= Math.abs(target.targetY - getY())) {
					horizontal = false; 
				}
			}
			// ajusta o rrumo do robo em 90 graus
			// caso contrario nove verticalmente
			if (horizontal) {
				newHeading -= Math.PI / 2;//90 graus
			}
			// Set the robot to turn left the amount of radians we have just calculated
			setTurnLeftRadians(Utils.normalRelativeAngle(newHeading));

			// Check if our robot has finished turning, i.e. has less than 1 degrees left to turn
			if (Math.abs(getTurnRemaining()) < 1 || Math.abs(getVelocity()) < 0.01) {
				// If we should move horizontally, the set the robot to move ahead with the
				// horizontal distance to the target robot. Otherwise, use the vertical distance.
				double delta; // delta is the delta distance to move
				if (horizontal) {
					delta = target.targetX - getX();
				} else {
					delta = target.targetY - getY();
				}
				setAhead(delta);

				// Set the new direction of our robot to 1 (meaning move forward) if the delta
				// distance is positive; otherwise it is set to -1 (meaning move backward).
				newDirection = delta > 0 ? 1 : -1;

				// Check if more than 10 turns have past since we changed the direction the last
				// time
				if (getTime() - lastDirectionShift > 10) {
					// If so, set the new direction to be the reverse direction if the velocity < 1
					if (Math.abs(getVelocity()) < 1) {
						newDirection = direction * -1;
					}
					// Check if the direction really changed
					if (newDirection != direction) {
						// If the new direction != current direction, then set the current direction
						// to be the new direction and save the current time so we know when we
						// changed the direction the last time.
						direction = newDirection;
						lastDirectionShift = getTime();
					}
				}
			}
		}
		// Set ahead 100 units forward or backward depending on the direction
		setAhead(100 * direction);
	}

	/**
	 * Method the updates the enemy map based on new scan data for a scanned robot.
	 * 
	 * @param scannedRobotEvent
	 *            is a ScannedRobotEvent event containing data about a scanned robot.
	 */
	private void updateEnemyMap(ScannedRobotEvent scannedRobotEvent) {
		// Gets the name of the scanned robot
		final String scannedRobotName = scannedRobotEvent.getName();

		// Get robot data for the scanned robot, if we have an entry in the enemy map
		RobotData scannedRobot = enemyMap.get(scannedRobotName);

		// Check if data entry exists for the scanned robot
		if (scannedRobot == null) {
			// No data entry exists => Create a new data entry for the scanned robot
			scannedRobot = new RobotData(scannedRobotEvent);
			// Put the new data entry into the enemy map
			enemyMap.put(scannedRobotName, scannedRobot);
		} else {
			// Data entry exists => Update the current entry with new scanned data
			scannedRobot.update(scannedRobotEvent);
		}
	}

	/**
	 * Method that updates the direction of the radar based on new scan data for a scanned robot.
	 * 
	 * @param scannedRobotEvent
	 *            is a ScannedRobotEvent event containing data about a scanned robot.
	 */
	private void updateScanDirection(ScannedRobotEvent scannedRobotEvent) {
		// Gets the name of the scanned robot
		final String scannedRobotName = scannedRobotEvent.getName();

		// Change the scanning direction if and only if we have no record for the oldest scanned
		// robot or the scanned robot IS the oldest scanned robot (based on the name) AND the enemy
		// map contains scanned data entries for ALL robots (the size of the enemy map is equal to
		// the number of opponent robots found by calling the getOthers() method).
		if ((oldestScanned == null || scannedRobotName.equals(oldestScanned.name)) && enemyMap.size() == getOthers()) {

			// Get the oldest scanned robot data from our LinkedHashMap, where the first value
			// contains the oldest accessed entry, which is the robot we need to get.
			RobotData oldestScannedRobot = enemyMap.values().iterator().next();

			// Get the recent scanned position (x,y) of the oldest scanned robot
			double x = oldestScannedRobot.scannedX;
			double y = oldestScannedRobot.scannedY;

			// Get the heading of our robot
			double ourHeading = getRadarHeadingRadians();

			// Calculate the bearing to the oldest scanned robot.
			// The bearing is the delta angle between the heading of our robot and the other robot,
			// which can be a positive or negative angle.
			double bearing = bearingTo(ourHeading, x, y);

			// Update the scan direction based on the bearing.
			// If the bearing is positive, the radar will be moved to the right.
			// If the bearing is negative, the radar will be moved to the left.
			scanDir = bearing;
		}
	}

	/**
	 * Updates the target positions for all enemies. The target position is the position our robot
	 * must fire at in order to hit the target robot. This robot uses Linear Targeting (Exact
	 * Non-iterative Solution) as described on the RoboWiki here:
	 * https://robowiki.net/wiki/Linear_Targeting
	 */
	private void updateEnemyTargetPositions() {
		// Go thru all robots in the enemy map
		for (RobotData enemy : enemyMap.values()) {

			// Variables prefixed with e- refer to enemy and b- refer to bullet
			double bV = Rules.getBulletSpeed(FIREPOWER);
			double eX = enemy.scannedX;
			double eY = enemy.scannedY;
			double eV = enemy.scannedVelocity;
			double eH = enemy.scannedHeading;

			// These constants make calculating the quadratic coefficients below easier
			double A = (eX - getX()) / bV;
			double B = (eY - getY()) / bV;
			double C = eV / bV * Math.sin(eH);
			double D = eV / bV * Math.cos(eH);

			// Quadratic coefficients: a*(1/t)^2 + b*(1/t) + c = 0
			double a = A * A + B * B;
			double b = 2 * (A * C + B * D);
			double c = (C * C + D * D - 1);

			// If the discriminant of the quadratic formula is >= 0, we have a solution meaning that
			// at some time, t, the bullet will hit the enemy robot if we fire at it now.
			double discrim = b * b - 4 * a * c;
			if (discrim >= 0) {
				// Reciprocal of quadratic formula. Calculate the two possible solution for the
				// time, t
				double t1 = 2 * a / (-b - Math.sqrt(discrim));
				double t2 = 2 * a / (-b + Math.sqrt(discrim));

				// Choose the minimum positive time or select the one closest to 0, if the time is
				// negative
				double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);

				// Calculate the target position (x,y) for the enemy. That is the point that our gun
				// should point at in order to hit the enemy at the time, t.
				double targetX = eX + eV * t * Math.sin(eH);
				double targetY = eY + eV * t * Math.cos(eH);

				// Assume enemy stops at walls. Hence, we limit that target position at the walls.
				double minX = HALF_ROBOT_SIZE;
				double minY = HALF_ROBOT_SIZE;
				double maxX = getBattleFieldWidth() - HALF_ROBOT_SIZE;
				double maxY = getBattleFieldHeight() - HALF_ROBOT_SIZE;

				enemy.targetX = limit(targetX, minX, maxX);
				enemy.targetY = limit(targetY, minY, maxY);
			}
		}
	}

	/**
	 * Updates which enemy robot from the enemy map that should be our current target.
	 */
	private void updateTarget() {
		// Set target to null, meaning that we have no target robot yet
		target = null;

		// Create a list over possible target robots that is a copy of robot data from the enemy map
		List<RobotData> targets = new ArrayList<RobotData>(enemyMap.values());

		// Run thru all the possible target robots and remove those that are outside the attack
		// range for this border sentry robot as our robot cannot do harm to robots outside its
		// range.
		Iterator<RobotData> it = targets.iterator();
		while (it.hasNext()) {
			RobotData robot = it.next();
			if (isOutsideAttackRange(robot.targetX, robot.targetY)) {
				it.remove();
			}
		}

		// Set the target robot to be the one among all possible target robots that is closest to
		// our robot.
		double minDist = Double.POSITIVE_INFINITY;
		for (RobotData robot : targets) {
			double dist = distanceTo(robot.targetX, robot.targetY);
			if (dist < minDist) {
				minDist = dist;
				target = robot;
			}
		}

		// If we still haven't got a target robot, then take the first one from our list of target
		// robots if the list is not empty.
		if (target == null && targets.size() > 0) {
			target = targets.get(0);
		}
	}

	/**
	 * Method that updates the gun direction to point at the current target.
	 */
	private void updateGunDirection() {
		// Only update the gun direction, if we have a current target
		if (target != null) {
			// Calculate the bearing between the gun and the target, which can be positive or
			// negative
			double targetBearing = bearingTo(getGunHeadingRadians(), target.targetX, target.targetY);
			// Set the gun to turn right the amount of radians defined by the bearing to the target
			setTurnGunRightRadians(targetBearing); // positive => turn right, negative => turn left
		}
	}

	/**
	 * Method that fires a bullet when the gun is ready to fire.
	 */
	private void fireGunWhenReady() {
		// We only fire the fun, when we have a target robot
		if (target != null) {
			// Only fire when the angle of the gun is pointing at our (virtual) target robot

			// Calculate the distance between between our robot and the target robot
			double dist = distanceTo(target.targetX, target.targetY);
			// Angle that "covers" the the target robot from its center to its edge
			double angle = Math.atan(HALF_ROBOT_SIZE / dist);

			// Check if the remaining angle (turn) to move the gun is less than our calculated cover
			// angle
			if (Math.abs(getGunTurnRemaining()) < angle) {
				// If so, our gun should be pointing at our target so we can hit it => fire!!
				setFire(FIREPOWER);
			}
		}
	}

	/**
	 * Method that checks if a coordinate (x,y) is outside the Border Sentry's attack range.
	 * 
	 * @param x
	 *            is the x coordinate.
	 * @param y
	 *            is the y coordinate.
	 * @return true if the coordinate is outside the attack range; false otherwise.
	 */
	private boolean isOutsideAttackRange(double x, double y) {
		double minBorderX = getSentryBorderSize();
		double minBorderY = getSentryBorderSize();
		double maxBorderX = getBattleFieldWidth() - getSentryBorderSize();
		double maxBorderY = getBattleFieldHeight() - getSentryBorderSize();

		return (x > minBorderX) && (y > minBorderY) && (x < maxBorderX) && (y < maxBorderY);
	}

	/**
	* Método que retorna um valor que está garantido dentro de um intervalo de valores definido por um
	* valor mínimo e máximo com base em um valor de entrada.
	* Se o valor de entrada for menor que o valor mínimo, o valor retornado será definido como
	*valor mínimo.
	* Se o valor de entrada for maior que o valor máximo, o valor retornado será definido como
	* valor maximo
	* Caso contrário, o valor retornado será igual ao valor de entrada.
	* @return o valor de entrada limitado que está garantido dentro do mínimo especificado e
	*/
	private double limit(double value, double min, double max) {
		return Math.min(max, Math.max(min, value));
	}

	/**
	* Métodos que retornam a distância até uma coordenada (x,y) do nosso robô.
	*/
	private double distanceTo(double x, double y) {
		return Math.hypot(x - getX(), y - getY());
	}

	/**
	* Método que retorna o ângulo para uma coordenada (x,y) do nosso robô.
	*/
	private double angleTo(double x, double y) {
		return Math.atan2(x - getX(), y - getY());
	}

	/**
	* Método que retorna o rumo para uma coordenada (x,y) da posição e rumo do nosso
	*robô. O rumo é o ângulo delta entre o rumo do nosso robô e o ângulo do
	* coordenada especificada.
	*/
	private double bearingTo(double heading, double x, double y) {
		return Utils.normalRelativeAngle(angleTo(x, y) - heading);
	}

	/**
	* Método que pinta um círculo preenchido na coordenada especificada (x,y) e na cor fornecida. O
	* o círculo terá um raio de 20 pixels (o que significa que o diâmetro será de 40 pixels).
	*/
	private void fillCircle(Graphics2D gfx, double x, double y, Color color) {
		// 	cor da caneta
		gfx.setColor(color);
	// Pinta um círculo preenchido (oval) com um raio de 20 pixels com centro na entrada
	// coordenadas.
		gfx.fillOval((int) x - 20, (int) y - 20, 40, 40);
	}

	/**
	* Esta classe é usada para armazenar dados sobre um robô que foi escaneado.
	* Os dados são principalmente um instantâneo de dados digitalizados específicos, como a posição digitalizada (x,y),
	* velocidade e rumo, coloque também a posição alvo prevista calculada do robô quando nosso
	*o robô precisa atirar no robô escaneado.<br>
	* Observe que esta classe calcula a posição (x,y) do robô escaneado conforme nosso robô se move,
	* e, portanto, dados como o ângulo e a distância até o robô escaneado mudarão com o tempo. por
	* usando a posição, é fácil calcular um novo ângulo e distância até o robô.
	*/
	class RobotData {
		final String name; // nome do robo escaneado
		double scannedX; // coordenada x baseado no ultimo scan
		double scannedY; // coordenada y naseado no ultimo scan
		double scannedVelocity; // velocidade baseado no ultimo scan
		double scannedHeading; // direção baseado no ultimo scan
		double targetX; // coordenada x do alvo
		double targetY; //coordenada y do alvo
		/**
		 * cria uma nova entrada de dados do robô com base em novos dados de varredura para um robô varrido.
		 */
		RobotData(ScannedRobotEvent event) {
			// armazena o nome do robo
			name = event.getName();
			// trata tudo como posição
			update(event);
			// inicializa as coordenadas (x,y) e senta o dedo na posicão
			targetX = scannedX;
			targetY = scannedY;
		}

		/**
		 * atualiza a posição pela novo escaneamento
		 */
		void update(ScannedRobotEvent event) {
			// pega a posição do robo escaneado
			Point2D.Double pos = getPosition(event);
			// armazena a posição escaneada (x,y)
			scannedX = pos.x;
			scannedY = pos.y;
			// armazena a velocidade e a direção
			scannedVelocity = event.getVelocity();
			scannedHeading = event.getHeadingRadians();
		}

		/**
		 * retorna a posição do robo scaneeado
		 */
		Point2D.Double getPosition(ScannedRobotEvent event) {
			// pega a distancia do robo escaneado
			double distance = event.getDistance();
			// calcula o angulo do robo escaneado
			double angle = getHeadingRadians() + event.getBearingRadians();

			// calcula as cordenadas  (x,y) do robo escadeado
			double x = getX() + Math.sin(angle) * distance;
			double y = getY() + Math.cos(angle) * distance;

			// retorna as posições  (x,y)
			return new Point2D.Double(x, y);
		}
	}
}