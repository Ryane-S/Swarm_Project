package com.example.interfacegraphique;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.image.ImageView;
import javafx.scene.image.Image;
import javafx.scene.layout.AnchorPane;
import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.text.Font;

public class HelloController {

    @FXML
    private AnchorPane root;

    @FXML
    private ImageView backgroundImage;

    @FXML
    private Button loadingButton;

    @FXML
    private Label label;

    @FXML
    public void initialize() {
        // Charger et afficher l'image de fond
        Image image = new Image(getClass().getResource("/images/fondApp.jpg").toExternalForm());
        backgroundImage.setImage(image);

        backgroundImage.setPreserveRatio(false); // Maintenir les proportions
        backgroundImage.setSmooth(true);
        backgroundImage.setCache(true);

        // Adapter l'image à la taille du root
        backgroundImage.fitWidthProperty().bind(root.widthProperty());
        backgroundImage.fitHeightProperty().bind(root.heightProperty());

        // Centrer le bouton dynamiquement quand la fenêtre change de taille
        root.widthProperty().addListener((obs, oldVal, newVal) -> updateButtonPosition());
        root.heightProperty().addListener((obs, oldVal, newVal) -> updateButtonPosition());

        // Lier la taille du bouton à celle de la fenêtre avec des limites
        bindButtonSize();

        // Positionner correctement le bouton au démarrage
        Platform.runLater(this::updateButtonPosition);

        // Initialiser la TextArea avec du texte par défaut
        bebou.setText("Entrez votre texte ici...");

        // Définir la police et la taille de la police
        bebou.setFont(Font.font("Arial", 14));


    }

    private void bindButtonSize() {
        // Lier la taille du bouton à celle de la fenêtre avec des limites
        double buttonWidthPercentage = 0.2; // 20% de la largeur
        double buttonHeightPercentage = 0.1; // 10% de la hauteur

        loadingButton.prefWidthProperty().bind(root.widthProperty().multiply(buttonWidthPercentage));
        loadingButton.prefHeightProperty().bind(root.heightProperty().multiply(buttonHeightPercentage));

        // Définir des limites pour la taille du bouton
        loadingButton.setMinWidth(100);
        loadingButton.setMaxWidth(200);
        loadingButton.setMinHeight(50);
        loadingButton.setMaxHeight(100);
    }

    private void updateButtonPosition() {
        double centerX = (root.getWidth() - loadingButton.getWidth()) / 2;
        double centerY = (root.getHeight() - loadingButton.getHeight()) / 2;

        loadingButton.setLayoutX(centerX);
        loadingButton.setLayoutY(centerY);
    }

    @FXML
    protected void onClickButton(){label.setText("William n'a pas répondu !");}



    @FXML
    private TextArea bebou;



        // Méthode pour changer dynamiquement la police et la taille de la police
        public void setFont(String fontName, double fontSize) {
            bebou.setFont(Font.font(fontName, fontSize));
    }

        // Autres méthodes pour personnaliser la TextArea...
    }
